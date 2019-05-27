package org.usfirst.frc.team3419.robot;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Solenoid;


import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.kauailabs.navx.frc.AHRS;

import constants.AlignerConstants;
import constants.DriveConstants;
import constants.GrabberConstants;
import constants.LEDConstants;
import constants.Ports;
import constants.RunConstants;
import constants.RunConstants.AngleMode;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import resource.ResourceFunctions;
import resource.Vector;
import robotcode.DriveTrain;
import robotcode.GearGrabberThreaded;
import robotcode.LedStrip;
import robotcode.RobotAngle;
import robotcode.Wheel;
import sensors.BooleanInputter;
import sensors.DoubleSolenoidInterface;
import sensors.MyGyroscope;
import sensors.RealDigitalInput;
import sensors.RealDoubleSolenoid;
import sensors.RotationInputter;
import sensors.SpeedInputter;
import sensors.TalonAbsoluteEncoder;
import sensors.TalonRelativeEncoder;

public class Robot extends SampleRobot {
	private RotationInputter[] mTurnEncoders;
	private SpeedInputter[] mDriveEncoders;
	private RealDoubleSolenoid mShifterSolenoid;

	private CANTalon[] mDrive;
	private CANTalon[] mTurn;

	private XboxController mXbox;
	private Joystick mSecondaryStick;

	private Wheel[] mWheels;
	private DriveTrain mDriveTrain;

	private AHRS mNavX;
	private AnalogGyro mWPIAnalogGyro;
	private MyGyroscope mCustomAnalogGyro;
	private AnalogInput mCustomAnalogGyroInput;

	private RobotAngle mRobotAngle;

	private CANTalon mWinchTalon;

	private CANTalon mGearAligner;
	private Thread mGearGrabberThread;
	private GearGrabberThreaded mGearGrabber;

	private BooleanInputter mBreakBeam, mLimitSwitch;

	private DoubleSolenoidInterface mFramePiston;
	private Solenoid mClamperPiston, mSnatcherPiston;
	private Compressor mCompressor;

	private final double kVoltsPerDegreePerSecond = 0.0068;


	private long mTimeStart;

	private Preferences mRobotPrefs;
	private PowerDistributionPanel mPDP;

	private long mCompressorTimeTotal, mCompressorTimeContinuous, mLastTimestampLogger;

	private UsbCamera mCamera;

	private DigitalOutput mLED_Data, mLED_Clock;
	private LedStrip mLEDStrip;
	private NetworkTable mJetsonTable;

	//@SuppressWarnings("unused")
	private void initializeReal() {
		if (RunConstants.RUN_MODE == RunConstants.RunMode.TEST_TALONS || RunConstants.RUN_MODE == RunConstants.RunMode.GYRO_TEST) {
			RunConstants.PNEUMATICS = false;
		}

		mTimeStart = System.currentTimeMillis();
		mCompressorTimeTotal = 0;
		mCompressorTimeContinuous = 0;
		mLastTimestampLogger = mTimeStart;

		mPDP = new PowerDistributionPanel();

		mRobotPrefs = Preferences.getInstance();

		//initializing arrays for wheels
		mTurnEncoders = new TalonAbsoluteEncoder[4];
		mDriveEncoders = new TalonRelativeEncoder[4];
		mShifterSolenoid = new RealDoubleSolenoid (Ports.SHIFTER_SOLENOID_IN, Ports.SHIFTER_SOLENOID_OUT);
		if (RunConstants.SHIFTY) mShifterSolenoid.set(DriveConstants.INITIAL_SPEED_FAST ? 
				DriveConstants.FAST_SHIFT_DIR : DriveConstants.SLOW_SHIFT_DIR);

		mWheels = new Wheel[4];	

		mTurn = new CANTalon[4];
		mDrive = new CANTalon[4];

		//initializing non-wheel elements
		mLED_Data = new DigitalOutput(Ports.LED_DATA);
		mLED_Clock = new DigitalOutput(Ports.LED_CLOCK);
		mLEDStrip = new LedStrip(mLED_Data, mLED_Clock, LEDConstants.NUM_LEDS, LEDConstants.LED_BRIGHTNESS);

		mJetsonTable = NetworkTable.getTable(AlignerConstants.TABLE_NAME);

		mCamera = CameraServer.getInstance().startAutomaticCapture("cam0", 0);
		mCamera.setResolution(160, 120);

		mCompressor = new Compressor();
		if (!(RunConstants.SHIFTY || RunConstants.PNEUMATICS)) mCompressor.stop();

		mXbox = new XboxController(0);
		mSecondaryStick = new Joystick (1);

		switch (RunConstants.ANGLE_MODE) {
		case NavX_USB:
			mNavX = new AHRS(Port.kUSB);
			mRobotAngle = new RobotAngle(mNavX, true, DriveConstants.NAVX_ANGLE_OFFSET);
			break;
		case NavX_I2C:
			mNavX = new AHRS(Port.kMXP);
			mRobotAngle = new RobotAngle(mNavX, true, DriveConstants.NAVX_ANGLE_OFFSET);
			break;
		case CustomAnalogGyro:
			mCustomAnalogGyroInput = new AnalogInput (Ports.ANALOG_GYRO_CHANNEL);
			mCustomAnalogGyro = new MyGyroscope(mCustomAnalogGyroInput, mCustomAnalogGyroInput, MyGyroscope.TWO_CANCEL);
			mCustomAnalogGyro.setSensitivity(kVoltsPerDegreePerSecond);
			mCustomAnalogGyro.initGyro();
			mRobotAngle = new RobotAngle (mCustomAnalogGyro, true, DriveConstants.NAVX_ANGLE_OFFSET);
			break;
		case WPIAnalogGyro:
			mWPIAnalogGyro = new AnalogGyro(Ports.ANALOG_GYRO_CHANNEL);
			mWPIAnalogGyro.setSensitivity(kVoltsPerDegreePerSecond);
			mWPIAnalogGyro.initGyro();
			mWPIAnalogGyro.setSensitivity(kVoltsPerDegreePerSecond);
			mRobotAngle = new RobotAngle (mWPIAnalogGyro, true, DriveConstants.NAVX_ANGLE_OFFSET);
			break;
		}


		mWinchTalon = new CANTalon (Ports.WINCH_TALON);
		mWinchTalon.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);

		mGearAligner = new CANTalon(Ports.GEAR_ALIGNER);

		mCompressor = new Compressor();
		if (!RunConstants.PNEUMATICS && !RunConstants.SHIFTY) mCompressor.stop();

		mFramePiston = new RealDoubleSolenoid (Ports.FRAME_PISTON_IN, Ports.FRAME_PISTON_OUT);
		mClamperPiston = new Solenoid (Ports.CLAMPER_PISTON);
		mSnatcherPiston = new Solenoid (Ports.SNATCHER_PISTON);

		mBreakBeam = new RealDigitalInput (Ports.BREAK_BEAM);
		mLimitSwitch = new RealDigitalInput (Ports.LIMIT_SWITCH);

		//initialize module elements
		for (int i = 0; i < 4; i++) {
			//motors
			mTurn[i] = new CANTalon (Ports.TURN[i]);
			mDrive[i] = new CANTalon (Ports.DRIVE[i]);
			mTurn[i].setInverted(DriveConstants.TURN_REVERSED[i]);

			//encoders
			mTurnEncoders[i] = new TalonAbsoluteEncoder(
					mTurn[i], DriveConstants.TURN_ENCODER_REVERSED[i], DriveConstants.ENCODER_OFFSETS[i]);

			mDriveEncoders[i] = new TalonRelativeEncoder(
					mDrive[i], DriveConstants.RAW_TO_RPS);
		}


		if (RunConstants.PNEUMATICS)
		{
			mGearGrabber = new GearGrabberThreaded (
					mBreakBeam, 
					mLimitSwitch, 
					mFramePiston, 
					mClamperPiston, 
					mSnatcherPiston, 
					mGearAligner, 
					mXbox);
			
		}
				//only initialize wheels when in swerve mode, otherwise their PIDs will activate
		if (RunConstants.RUN_MODE != RunConstants.RunMode.TEST_TALONS &&
				RunConstants.RUN_MODE != RunConstants.RunMode.ROTATE_TURN_MOTORS &&
				RunConstants.RUN_MODE != RunConstants.RunMode.GYRO_TEST) {
			for (int i = 0; i < 4; i++) {
				mWheels[i] = 
						new Wheel(mTurn[i], mDrive[i], 

								DriveConstants.ROTATION_PID_P[i], 
								DriveConstants.ROTATION_PID_I[i],
								DriveConstants.ROTATION_PID_D[i],
								DriveConstants.ROTATION_PID_IZONE[i],

								DriveConstants.FAST_SPEED_PID_P[i], 
								DriveConstants.FAST_SPEED_PID_I[i],
								DriveConstants.FAST_SPEED_PID_D[i],
								DriveConstants.FAST_SPEED_PID_F[i],

								DriveConstants.SLOW_SPEED_PID_P[i], 
								DriveConstants.SLOW_SPEED_PID_I[i],
								DriveConstants.SLOW_SPEED_PID_D[i],
								DriveConstants.SLOW_SPEED_PID_F[i],

								DriveConstants.DRIVE_ENCODER_REVERSED[i],

								DriveConstants.X_OFF[i],
								DriveConstants.Y_OFF[i], 

								mTurnEncoders[i], mDriveEncoders[i],
								DriveConstants.DRIVE_REVERSED[i]);
			}

			//instantiate control classes
			mDriveTrain = new DriveTrain(mWheels, mRobotAngle, mShifterSolenoid, mXbox, mSecondaryStick, mJetsonTable, mGearGrabber);	
		}
	}



	public Robot() {

	}

	@Override
	public void robotInit() {
		initializeReal();
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * if-else structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomous() {

	}

	public void performWinchCode() {
		while (isOperatorControl() && isEnabled()) {
			//handle subsytems
			mDriveTrain.drive();

			double speed = mXbox.getTriggerAxis(Hand.kRight) - mXbox.getTriggerAxis(Hand.kLeft);
			mWinchTalon.set(speed);

			Timer.delay(0.005);
		}
	}

	public void performFullCode() {
		while (isOperatorControl() && isEnabled()) {
			long timeStart = System.currentTimeMillis();
			//handle subsytems
			System.out.println( mTurnEncoders[1].getAngleDegrees());
			for (int i = 0; i < 4; i++) {
				double output = mDrive[i].getOutputVoltage() / mDrive[i].getBusVoltage();
				SmartDashboard.putNumber("Turn Encoder #" + i, mTurnEncoders[i].getAngleDegrees());
				SmartDashboard.putNumber("Drive Encoder #" + i, mDriveEncoders[i].getRPS());
				SmartDashboard.putNumber("Raw Drive Encoder #" + i, mDriveEncoders[i].getRawTicksPerSecond());
				SmartDashboard.putNumber("Drive PID Get #" + i, output);
				SmartDashboard.putNumber("Drive Setpoint #" + i, mDrive[i].getSetpoint() * DriveConstants.RAW_TO_RPS);
				SmartDashboard.putNumber("Drive F #" + i, mDrive[i].getF());
			}

			if (mSecondaryStick.getRawButton(6)) {
				mRobotAngle.reset();
			}
			if (mSecondaryStick.getRawButton(7) && RunConstants.ANGLE_MODE == AngleMode.CustomAnalogGyro) {
				mCustomAnalogGyro.initGyro();
				mCustomAnalogGyro.setSensitivity(kVoltsPerDegreePerSecond);
			}
			if (mSecondaryStick.getRawButton(7) && RunConstants.ANGLE_MODE == AngleMode.WPIAnalogGyro) {
				mWPIAnalogGyro.initGyro();
				mWPIAnalogGyro.calibrate();
				mWPIAnalogGyro.setSensitivity(kVoltsPerDegreePerSecond);

			}

			SmartDashboard.putNumber("Robot Angle:", mRobotAngle.getAngleDegrees());
			SmartDashboard.putNumber("Robot Angular Velocity:", mRobotAngle.getAngularVelocity());

			boolean runAutoGearPlacer = false; //mXbox.getBackButton();

			if (runAutoGearPlacer) {
				mCompressor.stop();
				mDriveTrain.align();
			}
			else {
				mCompressor.start();
				mDriveTrain.drive();
			}


			if (mXbox.getPOV() == 0) {
				mWinchTalon.set(-1.0);
			}
			else if (mXbox.getPOV() == 180) {
				mWinchTalon.set(0.5);
			}
			else if (mXbox.getBackButton()) {
				mWinchTalon.set(-0.3);
			}
			else {
				mWinchTalon.set(0.0);
			}

			//set LEDs using all these booleans
			boolean gearGrabbed = mGearGrabber.getGearGrabbed();
			//boolean breakBeamBroken = mBreakBeam.get() == GrabberConstants.GEAR_PRESENT_BREAK_BEAM;


			//boolean isActivelyAligning = mDriveTrain.isActivelyAligning();
			//boolean isActivelyPlacing = mDriveTrain.isActivelyPlacing();

			DriveTrain.AlignState alignState = mDriveTrain.getAlignState();
			SmartDashboard.putString("Vision Alignment State:", alignState.toString());
			SmartDashboard.putBoolean("Gear Grabbed:", gearGrabbed);

			mLEDStrip.fillColor(SmartDashboard.getBoolean("IsValid2", false) ? LEDConstants.GREEN : LEDConstants.RED);
			mLEDStrip.update();


			//boolean tryingToAlign = mDriveTrain.isActivelyAligning();
			//boolean isPlacing = mDriveTrain.isPegPlacing();

			logEverything();

			long timeEnd = System.currentTimeMillis();
			SmartDashboard.putNumber("Loop Time Millis:", timeEnd - timeStart);
			Timer.delay(0.005);
		}
	}


	/**
	 * Used to test wheel PIDs, turns wheels to an angle and drives in that direction
	 */
	public void performCrabDrive() {
		double prevAngle = 0;
		int whichTest = 4;
		while (isOperatorControl() && isEnabled()) {
			//mDriveTrain.iterate();
			Vector v = new Vector(mXbox.getX(Hand.kLeft), mXbox.getY(Hand.kLeft));
			double angle = -v.getAngle();
			angle -= 90;

			if (v.getMagnitude() < 0.5) {
				angle = prevAngle;
			}

			double speed = mXbox.getTriggerAxis(Hand.kRight) - mXbox.getTriggerAxis(Hand.kLeft);
			speed *= 0.4;

			double mag = Math.abs(mXbox.getX(Hand.kLeft) * mXbox.getX(Hand.kLeft) + mXbox.getY(Hand.kLeft) * mXbox.getY(Hand.kLeft));

			if (Math.abs(speed) < 0.05) speed = 0;


			double rotSpeed = (mXbox.getBumper(Hand.kLeft) ? 0.2 : 0) - 
					(mXbox.getBumper(Hand.kLeft) ? 0.2 : 0);

			if (mag > 0.03) {
				if (whichTest > 3) {
					for (int i = 0; i < 4; i++) {
						//if (i != 3) mWheels[i].set(0, 0);
						mWheels[i].set(angle, speed);
					}
				}
				else {
					mWheels[whichTest].set(angle, speed);
				}
			}
			else if (rotSpeed > 0) {
				mWheels[0].set(45, speed);
				mWheels[1].set(135, speed);
				mWheels[2].set(225, speed);
				mWheels[3].set(315, speed);
			}
			else {
				for (int i = 0; i < 4; i++) {
					mWheels[i].setSpeed(0);
				}
			}



			for (int i = 0; i < 4; i++) {
				SmartDashboard.putNumber("Encoder #" + i, mTurnEncoders[i].getAngleDegrees());
				SmartDashboard.putNumber("Turn #" + i, mTurn[i].get());
				SmartDashboard.putNumber("Drive #" + i, mDrive[i].get());
				SmartDashboard.putNumber("Setpoint #" + i, mWheels[i].getSetpoint());
				SmartDashboard.putNumber("Err #" + i, mWheels[i].getErr());
				SmartDashboard.putNumber("Output #" + i, mWheels[i].getPIDOutput());
			}

			Timer.delay(0.005);
		}
	}

	/**
	 * Used to find Talon ports, iterates through turn & drive motors with XboxController
	 */
	public void performMotorTest() {
		boolean yPressedPrev = false;
		boolean aPressedPrev = false;

		int motorIndex = 0;

		while (isOperatorControl() && isEnabled()) {

			if (yPressedPrev && !mXbox.getYButton()) motorIndex++;
			if (aPressedPrev && !mXbox.getAButton()) motorIndex--;

			yPressedPrev = mXbox.getYButton();
			aPressedPrev = mXbox.getAButton();

			motorIndex += 8; motorIndex %= 8;

			SmartDashboard.putNumber("Port:", motorIndex);
			SmartDashboard.putNumber("Robot Angle:", mRobotAngle.getAngleDegrees());
			System.out.println( mTurnEncoders[1].getAngleDegrees());

			for (int i = 0; i < 8; i++) {
				CANTalon t = i < 4 ? mTurn[i] : mDrive[i - 4];
				if (i == motorIndex) {	
					double speed = mXbox.getTriggerAxis(Hand.kRight) * 0.4 - mXbox.getTriggerAxis(Hand.kLeft) * 0.4;
					//System.out.println("Speed: " + speed);
					t.set(speed);
				}
				else {
					t.set(0);;
				}
			}

			for (int i = 0; i < 4; i++) {
				SmartDashboard.putNumber("Turn Encoder #" + i, mTurnEncoders[i].getAngleDegrees());
				SmartDashboard.putNumber("Drive Encoder #" + i, mDriveEncoders[i].getRPS());
			}

			if (mXbox.getXButton()) mGearAligner.set(0.5);
			else mGearAligner.set(0.0);

			SmartDashboard.putBoolean("Break Beam:", mBreakBeam.get() == GrabberConstants.GEAR_PRESENT_BREAK_BEAM);
			SmartDashboard.putBoolean("Limit Switch:", mLimitSwitch.get() == GrabberConstants.GEAR_PRESENT_LIMIT_SWITCH);

			Timer.delay(0.005); // wait for a motor update time
		}
	}

	private void performPneumaticsTest() {
		while (this.isOperatorControl() && this.isEnabled()) {
			if (mXbox.getBumper(Hand.kLeft)) mFramePiston.set(GrabberConstants.FRAME_UP);
			else mFramePiston.set(GrabberConstants.FRAME_DOWN);

			if (mXbox.getBumper(Hand.kRight)) mClamperPiston.set(GrabberConstants.CLAMPER_CLOSED);
			else mClamperPiston.set(GrabberConstants.CLAMPER_OPEN);

			if (mXbox.getAButton()) mSnatcherPiston.set(GrabberConstants.SNATCHER_CLOSED);
			else mSnatcherPiston.set(GrabberConstants.SNATCHER_OPEN);

			Timer.delay(0.005);
		}
	}

	private void performDriveConstantsPercent (double speed, int millis) {
		long timeStarted = -1;
		double angleStarted = mRobotAngle.getAngleDegrees();
		int prevDesiredAngle = -1;

		//for (int i = 0; i < 4; i++) mWheels[i].setNudgeMode(true);

		while (this.isOperatorControl() && this.isEnabled()) {
			int desiredAngle = -1;
			if (mXbox.getYButton()) desiredAngle = 0;
			else if (mXbox.getXButton()) desiredAngle = 90;
			else if (mXbox.getAButton()) desiredAngle = 180;
			else if (mXbox.getBButton()) desiredAngle = 270;

			if (desiredAngle != -1 && desiredAngle != prevDesiredAngle) {
				for (int i = 0; i < 4; i++) {
					mWheels[i].set(desiredAngle, 0);
				}
				Timer.delay(3.0); //let it get to the right angle

				timeStarted = System.currentTimeMillis();
				angleStarted = mRobotAngle.getAngleDegrees();
			}

			if (desiredAngle == -1) {
				for (int i = 0; i < 4; i++) {
					mWheels[i].setSpeed(0);
				}
			}

			else {
				long timeDif = System.currentTimeMillis() - timeStarted;
				if (timeDif < millis) {
					for (int i = 0; i < 4; i++) {
						mWheels[i].set(desiredAngle, speed);
					}
					//double dif = 
					double angleOff = ResourceFunctions.continuousAngleDif(mRobotAngle.getAngleDegrees(), angleStarted);

					SmartDashboard.putNumber("Angle Change:", angleOff);
				}
				else {
					for (int i = 0; i < 4; i++) {
						mWheels[i].set(desiredAngle, 0);
					}
				}

			}

			for (int i = 0; i < 4; i++) {
				SmartDashboard.putNumber("Turn Encoder #" + i, mTurnEncoders[i].getAngleDegrees());
				SmartDashboard.putNumber("Drive Encoder #" + i, mDriveEncoders[i].getRPS());
			}


			prevDesiredAngle = desiredAngle;
			Timer.delay(0.005);
		}
	}

	private void performRotateTurnMotors (double speed, int millis) {
		long timeStarted = -1;

		//for (int i = 0; i < 4; i++) mWheels[i].setNudgeMode(true);
		int prevDir = 0;
		double[] amountRotated = new double[] {0, 0, 0, 0};
		double[] prevAngle = new double[] {0, 0, 0, 0};

		while (this.isOperatorControl() && this.isEnabled()) {
			int desiredDir = 0;
			if (mXbox.getBumper(Hand.kLeft)) desiredDir = -1;
			else if (mXbox.getBumper(Hand.kRight)) desiredDir = 1;

			if (desiredDir != 0 && desiredDir != prevDir) {
				timeStarted = System.currentTimeMillis();
				for (int i = 0; i < 4; i++) {
					prevAngle[i] = mTurnEncoders[i].getAngleDegrees();
					amountRotated[i] = 0;
				}
			}

			if (desiredDir == 0) {
				for (int i = 0; i < 4; i++) {
					mTurn[i].set(0);
				}
			}
			else {
				long timeDif = System.currentTimeMillis() - timeStarted;
				if (timeDif < millis) {
					for (int i = 0; i < 4; i++) {
						mTurn[i].set(speed * desiredDir);
						double wheelAngle = mTurnEncoders[i].getAngleDegrees();
						amountRotated[i] += ResourceFunctions.continuousAngleDif(wheelAngle, prevAngle[i]);
						prevAngle[i] = wheelAngle;
					}

				}
				else {
					for (int i = 0; i < 4; i++) {
						mTurn[i].set(0);
					}
				}

			}


			for (int i = 0; i < 4; i++) {
				SmartDashboard.putNumber("Turn Encoder #" + i, mTurnEncoders[i].getAngleDegrees());
				SmartDashboard.putNumber("Drive Encoder #" + i, mDriveEncoders[i].getRPS());
				SmartDashboard.putNumber("Turned Degrees #" + i,  amountRotated[i]);
			}

			prevDir = desiredDir;


			Timer.delay(0.005);
		}
	}

	private void performRotateTimer() {
		long timeStarted = -1;
		int prevDesiredAngle = -1;

		//for (int i = 0; i < 4; i++) mWheels[i].setNudgeMode(true);

		while (this.isOperatorControl() && this.isEnabled()) {
			int desiredAngle = -1;
			if (mXbox.getYButton()) desiredAngle = 0;
			else if (mXbox.getXButton()) desiredAngle = 90;
			else if (mXbox.getAButton()) desiredAngle = 180;
			else if (mXbox.getBButton()) desiredAngle = 270;

			if (desiredAngle != -1 && desiredAngle != prevDesiredAngle) {
				timeStarted = System.currentTimeMillis();
			}

			if (desiredAngle != -1) {

				long timeDif = System.currentTimeMillis() - timeStarted;
				for (int i = 0; i < 4; i++) {
					mWheels[i].set(desiredAngle, 0);
					if (!mWheels[i].isInRangeNudge()) SmartDashboard.putNumber("Time to rotate #" + i, timeDif);
				}

			}

			for (int i = 0; i < 4; i++) {
				SmartDashboard.putNumber("Turn Encoder #" + i, mTurnEncoders[i].getAngleDegrees());
				SmartDashboard.putNumber("Drive Encoder #" + i, mDriveEncoders[i].getRPS());
			}


			prevDesiredAngle = desiredAngle;
			Timer.delay(0.005);
		}
	}

	public void performJetsonTest() {
		while (this.isOperatorControl() && this.isEnabled()) {
			double ul_x, ul_y, ur_x, ur_y, ll_x, ll_y, lr_x, lr_y;
			boolean valid;

			valid = mJetsonTable.getBoolean ("IsValid", false);
			ul_x = mJetsonTable.getNumber("UL_X", -1);
			ul_y = mJetsonTable.getNumber("UL_Y", -1);
			ur_x = mJetsonTable.getNumber("UR_X", -1);
			ur_y = mJetsonTable.getNumber("UR_Y", -1);
			ll_x = mJetsonTable.getNumber("LL_X", -1);
			ll_y = mJetsonTable.getNumber("LL_Y", -1);
			lr_x = mJetsonTable.getNumber("LR_X", -1);
			lr_y = mJetsonTable.getNumber("LR_Y", -1);

			SmartDashboard.putBoolean("is valid", valid);
			SmartDashboard.putNumber("ul x", ul_x);
			SmartDashboard.putNumber("ul y", ul_y);
			SmartDashboard.putNumber("ur x", ur_x);
			SmartDashboard.putNumber("ur y", ur_y);
			SmartDashboard.putNumber("ll x", ll_x);
			SmartDashboard.putNumber("ll y", ll_y);
			SmartDashboard.putNumber("lr x", lr_x);
			SmartDashboard.putNumber("lr y", lr_y);


			Timer.delay(0.005);
		}
	}

	public void performLEDTest() {
		while (this.isOperatorControl() && this.isEnabled()) {
			if (mXbox.getBumper(Hand.kRight)) {
				mLEDStrip.fillColor(LEDConstants.RED);
			}
			else if (mXbox.getBumper(Hand.kLeft)) {
				mLEDStrip.fillColor(LEDConstants.GREEN);
			}
			else if (mXbox.getAButton()) {
				mLEDStrip.fillColor(LEDConstants.CYAN);
			}
			else if (mXbox.getBButton()) {
				mLEDStrip.fillColor(LEDConstants.MAGENTA);
			}
			else if (mXbox.getYButton()) {
				mLEDStrip.fillColor(LEDConstants.YELLOW);
			}
			else {
				mLEDStrip.fillColor(LEDConstants.BLUE);
			}

			mLEDStrip.update();

			Timer.delay(0.005);
		}
	}

	public void performGyroTest() {
		while (this.isOperatorControl() && this.isEnabled()) {
			if (mSecondaryStick.getRawButton(6)) {
				mRobotAngle.reset();
			}
			if (mSecondaryStick.getRawButton(7) && RunConstants.ANGLE_MODE == AngleMode.CustomAnalogGyro) {
				mCustomAnalogGyro.initGyro();
				mCustomAnalogGyro.setSensitivity(kVoltsPerDegreePerSecond);
			}
			if (mSecondaryStick.getRawButton(7) && RunConstants.ANGLE_MODE == AngleMode.WPIAnalogGyro) {
				mWPIAnalogGyro.initGyro();
				mWPIAnalogGyro.calibrate();
				mWPIAnalogGyro.setSensitivity(kVoltsPerDegreePerSecond);

			}


			SmartDashboard.putNumber("Robot Angle:", mRobotAngle.getAngleDegrees());
			SmartDashboard.putNumber("Robot Angular Velocity:", mRobotAngle.getAngularVelocity());
		}
	}


	/**
	 * Runs the motors with arcade steering.
	 */
	@Override
	public void operatorControl() {
		startGame();

		switch (RunConstants.RUN_MODE) {
		case TEST_TALONS:
			performMotorTest();
			break;
		case FULL:
			performFullCode();
			break;
		case CRAB:
			performCrabDrive();
			break;
		case WINCH:
			performWinchCode();
			break;
		case TEST_PNEUMATICS:
			performPneumaticsTest();
			break;
		case DRIVE_CONSTANT_PERCENT:
			performDriveConstantsPercent(1.0, 2000);
			break;
		case ROTATE_TURN_MOTORS:
			performRotateTurnMotors(0.5, 5000);
			break;
		case ROTATE_TIMER:
			performRotateTimer ();
			break;
		case JETSON_TEST:
			performJetsonTest();
			break;
		case LED_TEST:
			performLEDTest();
			break;
		case GYRO_TEST:
			performGyroTest();
			break;
		}
	}

	@Override
	protected void disabled() {
		endGame();
	}

	public void endGame() {
		if (RunConstants.PNEUMATICS) {
			mGearGrabber.disable();
			mGearGrabberThread = null;
		}
	}

	//@SuppressWarnings("unused")
	public void startGame() {
		if (RunConstants.RUN_MODE != RunConstants.RunMode.TEST_TALONS && 
				RunConstants.RUN_MODE != RunConstants.RunMode.ROTATE_TURN_MOTORS &&	
				RunConstants.RUN_MODE != RunConstants.RunMode.GYRO_TEST &&	
				RunConstants.GET_FROM_SMARTDASHBOARD) {
			for (int i = 0; i < 4; i++) {
				DriveConstants.FAST_SPEED_PID_P[i] = mRobotPrefs.getDouble("FastPid_P" + i, DriveConstants.FAST_SPEED_PID_P[i]);
				DriveConstants.FAST_SPEED_PID_I[i] = mRobotPrefs.getDouble("FastPid_I" + i, DriveConstants.FAST_SPEED_PID_I[i]);
				DriveConstants.FAST_SPEED_PID_D[i] = mRobotPrefs.getDouble("FastPid_D" + i, DriveConstants.FAST_SPEED_PID_D[i]);
				DriveConstants.FAST_SPEED_PID_F[i] = mRobotPrefs.getDouble("FastPid_F" + i, DriveConstants.FAST_SPEED_PID_F[i]);

				DriveConstants.SLOW_SPEED_PID_P[i] = mRobotPrefs.getDouble("SlowPid_P" + i, DriveConstants.SLOW_SPEED_PID_P[i]);
				DriveConstants.SLOW_SPEED_PID_I[i] = mRobotPrefs.getDouble("SlowPid_I" + i, DriveConstants.SLOW_SPEED_PID_I[i]);
				DriveConstants.SLOW_SPEED_PID_D[i] = mRobotPrefs.getDouble("SlowPid_D" + i, DriveConstants.SLOW_SPEED_PID_D[i]);
				DriveConstants.SLOW_SPEED_PID_F[i] = mRobotPrefs.getDouble("SlowPid_F" + i, DriveConstants.SLOW_SPEED_PID_F[i]);


				mWheels[i].setPIDVals(
						DriveConstants.FAST_SPEED_PID_P[i], 
						DriveConstants.FAST_SPEED_PID_I[i], 
						DriveConstants.FAST_SPEED_PID_D[i], 
						DriveConstants.FAST_SPEED_PID_F[i], 

						DriveConstants.SLOW_SPEED_PID_P[i], 
						DriveConstants.SLOW_SPEED_PID_I[i], 
						DriveConstants.SLOW_SPEED_PID_D[i], 
						DriveConstants.SLOW_SPEED_PID_F[i]);
			}
		}

		if (RunConstants.PNEUMATICS) {
			mGearGrabber.enable();
			mGearGrabberThread = new Thread (mGearGrabber);
			mGearGrabberThread.start();
		}
	}

	/**
	 * Runs during test mode
	 */
	@Override
	public void test() {
	}

	public void logEverything() {

		//current
		for (int i = 0; i < 4; i++) {
			SmartDashboard.putNumber ("Turn" + (i+1) + "_Current", mTurn[i].getOutputCurrent());
			SmartDashboard.putNumber ("Drive" + (i+1) + "_Current", mDrive[i].getOutputCurrent());
			SmartDashboard.putNumber ("Turn" + (i+1) + "_Voltage", mTurn[i].getOutputVoltage());
			SmartDashboard.putNumber ("Drive" + (i+1) + "_Voltage", mDrive[i].getOutputVoltage());
		}

		//note: make sure pressureSwitchValue() is actually telling us if it is on

		boolean compressorOn = mCompressor.getPressureSwitchValue();
		long timestamp = System.currentTimeMillis() - mTimeStart;
		if (compressorOn) {
			mCompressorTimeTotal += timestamp - mLastTimestampLogger;
			mCompressorTimeContinuous += timestamp - mLastTimestampLogger;
		}
		else {
			mCompressorTimeContinuous = 0;
		}

		SmartDashboard.putNumber("TotalCurrent", mPDP.getTotalCurrent());
		SmartDashboard.putNumber("TotalVoltage", mPDP.getVoltage());
		SmartDashboard.putNumber("CompressorCurrent", mCompressor.getCompressorCurrent());
		SmartDashboard.putNumber("CompressorOn", compressorOn ? 1 : 0);
		SmartDashboard.putNumber("CompressorTimeTotal", mCompressorTimeTotal);
		SmartDashboard.putNumber("CompressorTimeContinuous", mCompressorTimeContinuous);


		SmartDashboard.putNumber ("Timestamp", timestamp);
		mLastTimestampLogger = timestamp;
	}
}


/*
switch (alignState) {
//gear hunting
case NONE:
	if (gearGrabbed) {
		mLEDStrip.fillColor(LEDConstants.GREEN);
	}
	else if (breakBeamBroken) {
		mLEDStrip.fillColor(LEDConstants.YELLOW);
	}
	else {
		mLEDStrip.fillColor(LEDConstants.RED);
	}
	break;
case SEARCHING:
	mLEDStrip.fillColor (LEDConstants.RED);
	break;
case ALIGN_TO_PEG:
	mLEDStrip.fillColor (LEDConstants.YELLOW);
	break;
case FORWARD:
	mLEDStrip.fillColor (LEDConstants.GREEN);
	break;
case RELEASING:
	mLEDStrip.fillColor (LEDConstants.GREEN);
	break;
case BACKWARD:
	mLEDStrip.fillColor (LEDConstants.GREEN);
	break;
case COMPLETED:
	mLEDStrip.fillColor (LEDConstants.GREEN);
	break;
}
 */
/*if (gearGrabbed) {
	mLEDStrip.fillColor(LEDConstants.GREEN);
}
else if (breakBeamBroken) {
	mLEDStrip.fillColor(LEDConstants.YELLOW);
}
else {
	mLEDStrip.fillColor(LEDConstants.RED);
}*/