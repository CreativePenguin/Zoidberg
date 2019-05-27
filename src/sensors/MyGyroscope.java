package sensors;


import constants.Configurables;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
import edu.wpi.first.wpilibj.tables.ITable;

/**
 * Use a rate gyro to return the robots heading relative to a starting position.
 * The Gyro class tracks the robots heading based on the starting position. As
 * the robot rotates the new heading is computed by integrating the rate of
 * rotation returned by the sensor. When the class is instantiated, it does a
 * short calibration routine where it samples the gyro while at rest to
 * determine the default offset. This is subtracted from each sample to
 * determine the heading.
 */
//SIM
public class MyGyroscope extends SensorBase implements PIDSource, LiveWindowSendable {

	static final int kOversampleBits = 10;
	static final int kAverageBits = 0;
	static final double kSamplesPerSecond = 50.0;
	static final double kCalibrationSampleTime = Configurables.GYRO_SAMPLE_TIME;
	static final double kDefaultVoltsPerDegreePerSecond = 0.007;
	static final double kCancelDeadband = 0.01;
	
	public static final int TWO_CANCEL = 1, 
							TWO_DEADBAND = 2;
	
	protected double mOffset;
	
	protected int mMode;
	protected double mAccumulatedAngle;
	
	protected AnalogInput m_analog1;
	protected AnalogInput m_analog2;
	protected AnalogInput[] mAnalogs;
	double m_voltsPerDegreePerSecond;
	double[] m_offsets;
	int[] m_centers;
	boolean m_channelAllocated = false;
	AccumulatorResult result;

	/**
	 * Initialize the gyro. Calibrate the gyro by running for a number of
	 * samples and computing the center value. Then use the center
	 * value as the Accumulator center value for subsequent measurements. It's
	 * important to make sure that the robot is not moving while the centering
	 * calculations are in progress, this is typically done when the robot is
	 * first turned on while it's sitting at rest before the competition starts.
	 */
	public void initGyro() {
		result = new AccumulatorResult();
		if (m_analog1 == null || m_analog2 == null) {
			System.out.println("Null m_analog");
		}
		m_voltsPerDegreePerSecond = kDefaultVoltsPerDegreePerSecond;
		
		
		m_analog1.setAverageBits(kAverageBits);
		m_analog1.setOversampleBits(kOversampleBits);
		
		m_analog2.setAverageBits(kAverageBits);
		m_analog2.setOversampleBits(kOversampleBits);
		
		
		double sampleRate = kSamplesPerSecond
				* (1 << (kAverageBits + kOversampleBits));
		AnalogInput.setGlobalSampleRate(sampleRate);
		Timer.delay(1.0);

		m_analog1.initAccumulator();
		m_analog1.resetAccumulator();
		m_analog2.initAccumulator();
		m_analog2.resetAccumulator();

		Timer.delay(kCalibrationSampleTime);

		for(int i =0 ; i < mAnalogs.length; i++)
		{
			mAnalogs[i].getAccumulatorOutput(result);
	
			m_centers[i]= (int) ((double) result.value / (double) result.count + .5);
	
			m_offsets[i] = ((double) result.value / (double) result.count)
					- m_centers[i];
	
			mAnalogs[i].setAccumulatorCenter(m_centers[i]);
			mAnalogs[i].resetAccumulator();
	
			
			//LiveWindow.addSensor("Gyro", m_analog.getChannel(), this);
		}
		if(mMode == TWO_CANCEL) setDeadband(kCancelDeadband);
		else if(mMode == TWO_DEADBAND) {
			setDeadband1(0.0);
			mAccumulatedAngle = 0;
		}
		
		mOffset = 0;
	}


	/**
	 * Gyro constructor with a precreated analog channel object. Use this
	 * constructor when the analog channel needs to be shared.
	 *
	 * @param channel
	 *            The AnalogInput object that the gyro is connected to. Gyros 
	              can only be used on on-board channels 0-1.
	 */
	public MyGyroscope(AnalogInput channel1, AnalogInput channel2, int mode) {
		this.mMode = mode;
		m_analog1 = channel1;
		m_analog2 = channel2;
		if (m_analog1 == null || m_analog2==null) {
			throw new NullPointerException("AnalogInput supplied to Gyro constructor is null");
		}
		mAnalogs = new AnalogInput[2];
		mAnalogs[0]= m_analog1;
		mAnalogs[1]=m_analog2;
		m_centers = new int[2];
		m_offsets = new double[2];
	}

	/**
	 * Reset the gyro. Resets the gyro to a heading of zero. This can be used if
	 * there is significant drift in the gyro and it needs to be recalibrated
	 * after it has been running.
	 */
	public void reset() {
		for(AnalogInput m_analog : mAnalogs)
		{
			if (m_analog != null) {
				m_analog.resetAccumulator();
			}
		}
	}

	/**
	 * Delete (free) the accumulator and the analog components used for the
	 * gyro.
	 */
	@Override
	public void free() {
		for(AnalogInput m_analog : mAnalogs)
		{
			if (m_analog != null && m_channelAllocated) {
				m_analog.free();
			}			
		}
		for(int i = 0; i < mAnalogs.length; i++)
		{
			mAnalogs[i] = null;
		}
	}
	
	public double getAngle(int pSensorNumber)
	{
		
		if (mAnalogs[pSensorNumber] == null ) {
			return 90.0;
		} else {
			mAnalogs[pSensorNumber].getAccumulatorOutput(result);

			long value = result.value - (long) (result.count * m_offsets[pSensorNumber]);

			double scaledValue = value
					* 1e-9
					* mAnalogs[pSensorNumber].getLSBWeight()
					* (1 << mAnalogs[pSensorNumber].getAverageBits())
					/ (AnalogInput.getGlobalSampleRate() * m_voltsPerDegreePerSecond);
			
			return (((-scaledValue) + 90.0) + 36000.0) % 360.0;
		}
	}
	
	public double getRate(int pSensorNumber)
	{
		if (mAnalogs[pSensorNumber] == null ) {
			return 0;
		} else {
			return
			 (mAnalogs[pSensorNumber].getAverageValue() - (m_centers[pSensorNumber] + m_offsets[pSensorNumber]))
					* 1e-9
					* mAnalogs[pSensorNumber].getLSBWeight()
					/ ((1 << mAnalogs[pSensorNumber].getOversampleBits()) * m_voltsPerDegreePerSecond);
			
		}

	}

	/**
	 * Return the actual angle in degrees that the robot is currently facing.
	 *
	 * The angle is based on the current accumulator value corrected by the
	 * oversampling rate, the gyro type and the A/D calibration values. The
	 * angle is continuous, that is it will continue from 360 to 361 degrees. This allows
	 * algorithms that wouldn't want to see a discontinuity in the gyro output
	 * as it sweeps past from 360 to 0 on the second time around.
	 *
	 * @return the current heading of the robot in degrees. This heading is
	 *         based on integration of the returned rate from the gyro.
	 */
	public double getRawAngle(int pSensorNumber) {
		
		
		
			mAnalogs[pSensorNumber].getAccumulatorOutput(result);

			long value = result.value - (long) (result.count * m_offsets[pSensorNumber]);

			double scaledValue = value
					* 1e-9
					* mAnalogs[pSensorNumber].getLSBWeight()
					* (1 << mAnalogs[pSensorNumber].getAverageBits())
					/ (AnalogInput.getGlobalSampleRate() * m_voltsPerDegreePerSecond);
			
			return scaledValue;
		}
	
	public double getAngle()
	{
		if(mMode == TWO_CANCEL) {
			double val = (getRawAngle(0) + getRawAngle(1)) / 2.0;
			val += mOffset;
			return (((-val)) + 36000.0) % 360.0;
		}
		else {
			return 0;
		}
	}

	/**
	 * Return the rate of rotation of the gyro
	 *
	 * The rate is based on the most recent reading of the gyro analog value
	 *
	 * @return the current rate in degrees per second
	 */
	public double getRate() {
		return (getRate(0)+getRate(1))/2.0;
	}

	/**
	 * Set the gyro sensitivity. This takes the number of
	 * volts/degree/second sensitivity of the gyro and uses it in subsequent
	 * calculations to allow the code to work with multiple gyros. This value
	 * is typically found in the gyro datasheet.
	 *
	 * @param voltsPerDegreePerSecond
	 *            The sensitivity in Volts/degree/second.
	 */
	public void setSensitivity(double voltsPerDegreePerSecond) {
		m_voltsPerDegreePerSecond = voltsPerDegreePerSecond;
	}

	/**
	 * Set the size of the neutral zone.  Any voltage from the gyro less than
	 * this amount from the center is considered stationary.  Setting a
	 * deadband will decrease the amount of drift when the gyro isn't rotating,
	 * but will make it less accurate.
	 *
	 * @param volts The size of the deadband in volts
	 */
	void setDeadband(double volts) {
		for(AnalogInput m_analog : mAnalogs)
		{
			int deadband = (int)(volts * 1e9 / m_analog.getLSBWeight() * (1 << m_analog.getOversampleBits()));
			m_analog.setAccumulatorDeadband(deadband);
		}
	}
	
	void setDeadband1(double volts) {
		int deadband = (int)(volts * 1e9 / m_analog1.getLSBWeight() * (1 << m_analog1.getOversampleBits()));
		m_analog1.setAccumulatorDeadband(deadband);		
	}
	
	void setDeadband2(double volts) {
		int deadband = (int)(volts * 1e9 / m_analog2.getLSBWeight() * (1 << m_analog2.getOversampleBits()));
		m_analog2.setAccumulatorDeadband(deadband);		
	}

	public void setOffset(double offset) {
		mOffset = offset;
	}

	/**
	 * Get the output of the gyro for use with PIDControllers.
	 * May be the angle or rate depending on the set PIDSourceParameter
	 *
	 * @return the output according to the gyro
	 */
	@Override
	public double pidGet() {
		/*switch (m_pidSource.value) {
		case 1://PIDSourceParameter.kRate_val:
			return getRate();
		case 2:// PIDSourceParameter.kAngle_val:
			return getAngle();
		default:
			return 0.0;
		}*/
		return getAngle();
	}

	/*
	 * Live Window code, only does anything if live window is activated.
	 */
	@Override
	public String getSmartDashboardType() {
		return "Gyro";
	}

	private ITable m_table;

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void initTable(ITable subtable) {
		m_table = subtable;
		updateTable();
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public ITable getTable() {
		return m_table;
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void updateTable() {
		if (m_table != null) {
			m_table.putNumber("Value", getAngle());
		}
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void startLiveWindowMode() {
	}

	/**
	 * {@inheritDoc}
	 */
	@Override
	public void stopLiveWindowMode() {
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		// TODO Auto-generated method stub
		return null;
	}
}
