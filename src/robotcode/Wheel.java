package robotcode;

import resource.ResourceFunctions;
import resource.Vector;
import sensors.RotationInputter;
import sensors.SpeedInputter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import constants.DriveConstants;
import constants.RunConstants;
import edu.wpi.first.wpilibj.LocalPIDController;

public class Wheel {
	
	private CANTalon mTurn;
	private CANTalon mDrive;
	
	private double m_xOff;
	private double m_yOff;

	private LocalPIDController mRotationPID;
	
	private RotationInputter mRotationSource;
	private SpeedInputter mSpeedSource;
	
	private boolean mReverseSpeed;
	
	private boolean mLastWasInRange;
	private double mFirstTimeInRange;
	
	private boolean mFastMode;
	
	private double 
		mFastSpeedPID_P, mFastSpeedPID_I, mFastSpeedPID_D, mFastSpeedPID_F,
		mSlowSpeedPID_P, mSlowSpeedPID_I, mSlowSpeedPID_D, mSlowSpeedPID_F;
//	
//	private int
//		mFastSpeedPID_IZone, mSlowSpeedPID_IZone;
	
	public Wheel(CANTalon pTurn, CANTalon pDrive, 
			double pROT_PID_P, double pROT_PID_I, double pROT_PID_D, double pROT_PID_IZONE,
			double pFAST_SPEED_PID_P, double pFAST_SPEED_PID_I, double pFAST_SPEED_PID_D,  double pFAST_SPEED_PID_F,
			double pSLOW_SPEED_PID_P, double pSLOW_SPEED_PID_I, double pSLOW_SPEED_PID_D,  double pSLOW_SPEED_PID_F,
			boolean pREVERSE_DRIVE_ENCODER,
			double pXOff, double pYOff,
			RotationInputter pTurnEncoder, SpeedInputter pDriveEncoder, 
			boolean pReversed) {
		
		//initialize motors
		mTurn = pTurn;
		mDrive = pDrive;
		
		//initialize wheel position from robot center
		m_xOff = pXOff;
		m_yOff = pYOff;
		
		//initialize encoder
		mRotationSource = pTurnEncoder;
		mSpeedSource = pDriveEncoder;
		
		//initialize turn motor PID
		mRotationPID = new LocalPIDController(pROT_PID_P, pROT_PID_I, pROT_PID_D, mRotationSource, mTurn);
		mRotationPID.setContinuous(true); //wrap around 360
		mRotationPID.setInputRange(0, 360);
		mRotationPID.setOutputRange(-1, 1); //may want to set nominal output for minimums
		mRotationPID.setIZone(pROT_PID_IZONE);
		mRotationPID.enable();

		mDrive.reverseSensor(pREVERSE_DRIVE_ENCODER);
		mDrive.setInverted(pReversed);
		
		mFastSpeedPID_P = pFAST_SPEED_PID_P;
		mFastSpeedPID_I = pFAST_SPEED_PID_I;
		mFastSpeedPID_D = pFAST_SPEED_PID_D;
		mFastSpeedPID_F = pFAST_SPEED_PID_F;
		//mFastSpeedPID_IZone = pFAST_SPEED_PID_IZONE;
		
		mSlowSpeedPID_P = pSLOW_SPEED_PID_P;
		mSlowSpeedPID_I = pSLOW_SPEED_PID_I;
		mSlowSpeedPID_D = pSLOW_SPEED_PID_D;
		mSlowSpeedPID_F = pSLOW_SPEED_PID_F;
		//mSlowSpeedPID_IZone = pSLOW_SPEED_PID_IZONE;

		//handles nudging logic
		mLastWasInRange = false;
		mFirstTimeInRange = 0;
		
		mFastMode = DriveConstants.INITIAL_SPEED_FAST;
		
		//initialize drive motor PID
		if (RunConstants.SPEED_PID) {
//			mSpeedPID = new LocalPIDController(pSPEED_PID_P, pSPEED_PID_I, pSPEED_PID_D, pSPEED_PID_F, mSpeedSource, mDrive);
//			mSpeedPID.setContinuous(false);
//			mSpeedPID.setInputRange(-DriveConstants.MAX_RPS * 10, DriveConstants.MAX_RPS * 10);
//			mSpeedPID.setOutputRange(-1, 1);
//			mSpeedPID.setSetpoint(0);
//			mSpeedPID.enable();
			

			mDrive.setProfile(0);
			setPIDConstants ();
			
			mDrive.configNominalOutputVoltage(+0.0f, -0.0f);
			mDrive.configPeakOutputVoltage(+12.0f, -12.0f);
			mDrive.setSetpoint(0);
			mDrive.setCloseLoopRampRate(0);
			mDrive.changeControlMode(TalonControlMode.Speed);
			mDrive.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
			mDrive.enableControl();
			mDrive.enable();
			
		}
		else {
			mDrive.changeControlMode(TalonControlMode.PercentVbus);
		}
	}
	
	public void setPIDVals(
			double pFAST_SPEED_PID_P,
			double pFAST_SPEED_PID_I,
			double pFAST_SPEED_PID_D,
			double pFAST_SPEED_PID_F, 
			
			double pSLOW_SPEED_PID_P, 
			double pSLOW_SPEED_PID_I, 
			double pSLOW_SPEED_PID_D, 
			double pSLOW_SPEED_PID_F) {
		mFastSpeedPID_P = pFAST_SPEED_PID_P;
		mFastSpeedPID_I = pFAST_SPEED_PID_I;
		mFastSpeedPID_D = pFAST_SPEED_PID_D;
		mFastSpeedPID_F = pFAST_SPEED_PID_F;
		//mFastSpeedPID_IZone = pFAST_SPEED_PID_IZONE;
		
		mSlowSpeedPID_P = pSLOW_SPEED_PID_P;
		mSlowSpeedPID_I = pSLOW_SPEED_PID_I;
		mSlowSpeedPID_D = pSLOW_SPEED_PID_D;
		mSlowSpeedPID_F = pSLOW_SPEED_PID_F;
		//mSlowSpeedPID_IZone = pSLOW_SPEED_PID_IZONE;
		setPIDConstants();
	}
	
	private void setPIDConstants() {
		SmartDashboard.putString("Setting pid constants", "!!!!!" + " " + mFastSpeedPID_F);
		if (mFastMode) {
			mDrive.setP(mFastSpeedPID_P / DriveConstants.RAW_TO_RPS);
			mDrive.setI(mFastSpeedPID_I / DriveConstants.RAW_TO_RPS);
			mDrive.setD(mFastSpeedPID_D / DriveConstants.RAW_TO_RPS);
			mDrive.setF(mFastSpeedPID_F / DriveConstants.RAW_TO_RPS);
			//mDrive.setIZone(mFastSpeedPID_IZone / DriveConstants.RAW_TO_RPS);
		}
		
		else {
			mDrive.setP(mSlowSpeedPID_P / DriveConstants.RAW_TO_RPS);
			mDrive.setI(mSlowSpeedPID_I / DriveConstants.RAW_TO_RPS);
			mDrive.setD(mSlowSpeedPID_D / DriveConstants.RAW_TO_RPS);
			mDrive.setF(mSlowSpeedPID_F / DriveConstants.RAW_TO_RPS);
			//mDrive.setIZone(mSlowSpeedPID_IZone);
		}
	}
	
	public void setSpeedMode (boolean pFastMode) {
		boolean eq = mFastMode == pFastMode;
		mFastMode = pFastMode;
		if (!eq) setPIDConstants();
	}
	
	
	/**
	 * Calculates if we are currently within nudging tolerance
	 * @return true if we are within nudge tolerance, otherwise false
	 */
	@SuppressWarnings("unused")
	public boolean isInRangeNudge() {
		if (DriveConstants.MIN_TIME_IN_RANGE_NUDGE_MILLIS <= 0 ) {
			return Math.abs(mRotationPID.getError()) < DriveConstants.NUDGE_ANGLE_TOLERANCE;
		}
		else {
			if(Math.abs(mRotationPID.getError()) < DriveConstants.NUDGE_ANGLE_TOLERANCE) {
				if(!mLastWasInRange) {
					mLastWasInRange = true;
					mFirstTimeInRange = System.currentTimeMillis();
					return false;
				}
				else {
					mLastWasInRange = true;
					return (System.currentTimeMillis() - mFirstTimeInRange) > DriveConstants.MIN_TIME_IN_RANGE_NUDGE_MILLIS;
				}
			}
			else {
				mLastWasInRange = false;
				return false;
			}
		}
		
	}
	
	
	public double getSetpoint() {
		return mRotationPID.getSetpoint();
	}
	
	public double getErr() {
		return mRotationPID.getError();
	}
	
	public double getPIDOutput() {
		return mRotationPID.get();
	}
	
	public double getRealAngle() {
		return mRotationSource.pidGet();
	}

	public Vector getVec() {
		return Vector.createPolar(getRealAngle(), Math.abs(getSpeed()));
	}
	
	public double getSpeed() {
		return mSpeedSource.getRPS();
	}
	
	
	/**
	 * Gets x-distance from the center of the robot
	 * x-axis points NORTH relative to the robot
	 * @return x-distance in inches
	 */
	public double getXOff() {
		return m_xOff;
	}
	
	/**
	 * Gets the y-distance from the center of the robot
	 * y-axis points WEST relative to the robot
	 * @return y-distance in inches
	 */
	public double getYOff() {
		return m_yOff;
	}
	
	/**
	 * Set wheel angle & speed
	 * @param pWheelVelocity Vector of wheel velocity
	 */
	public void set (Vector pWheelVelocity) {
		set (pWheelVelocity.getAngle(), pWheelVelocity.getMagnitude());
	}
	
	/**
	 * Set wheel angle & speed
	 * @param angle direction to point the wheel
	 * @param speed magnitude to drive the wheel
	 */
	public void set(double angle, double speed) {
		setAngle(angle);
		setSpeed(speed);
	}
	
	/**
	 * Sets the wheel angle optimizing for travel time
	 * tells a PID what setpoint to use
	 * @param angle direction to point the wheel
	 */
	public void setAngle(double angle) {
		//find distance from setpoint (as the PID would calculate it)
		double dif = mRotationPID.getSetpoint() - angle;
		dif = ResourceFunctions.putAngleInRange(dif);
		if(dif > 180) dif = 360 - dif;

		//reverse the wheel direction & add 180 to encoder angle if its faster than turning around
		if(dif > 100) {
			mReverseSpeed = !mReverseSpeed; 
			mRotationSource.setAdd180(mReverseSpeed);
		}

		mRotationPID.setSetpoint(ResourceFunctions.putAngleInRange(angle));
	}
	
	/**
	 * Sets the speed of the wheel including multipliers
	 * @param speed raw speed of the wheel
	 */
	public void setSpeed(double speed) {
		double realSpeed = speed;
		realSpeed = mReverseSpeed ? -realSpeed : realSpeed;
		
		if (RunConstants.SPEED_PID) {
			double rps = realSpeed;
			if (mFastMode) rps *= DriveConstants.MAX_RPS_FAST_MODE;
			else rps *= DriveConstants.MAX_RPS_SLOW_MODE;
			
			double rawSpeedUnits = rps / DriveConstants.RAW_TO_RPS;
			
			mDrive.set(rawSpeedUnits);
		}
		else {
			mDrive.set(realSpeed);
		}
	}
}
