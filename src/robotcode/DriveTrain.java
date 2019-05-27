package robotcode;

import constants.AlignerConstants;
import constants.Configurables;
import constants.DriveConstants;
import constants.RunConstants;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.LocalPIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import resource.ResourceFunctions;
import resource.Vector;
import sensors.DoubleSolenoidInterface;

/**
 * Controls the swerve drive in all control modes
 * Uses the XbxoController and visual feedback to determine 
 * high-level state decisions such as robot's linear and angular velocity
 * Individual wheel velocities are then calculated and achieved seperately
 * @author 3419
 *
 */
public class DriveTrain {
	private SwerveDrive mSwerveDrive;
	private Wheel[] mWheels;
	private RobotAngle mRobotAngle;
	
	private LocalPIDController mGyroPID;
	private GenericPIDOutput mGyroPID_Output;
	
	private XboxController mController;
	private Joystick mSecondaryStick;
	
	private DoubleSolenoidInterface mShifterSolenoid;
	private long mTimeStoppedTurning;
	
	private LocalPIDController mLR_PID, mAlignAngle_PID;
	private GenericPIDOutput mLR_PIDOutput, mAlignAngle_PIDOutput;
	private GenericPIDInput mLR_PIDInput, mAlignAngle_PIDInput;
	
	private NetworkTable mJetsonTable;
	private AlignState mAlignState;
	private long mFirstTimeAligned;
	
	private GearGrabberThreaded mGearGrabber;
	private long mTimeStartedForward;
	private long mTimeStartedBackward;
	
	public DriveTrain (Wheel[] pWheels, 
			RobotAngle pRobotAngle,
			DoubleSolenoidInterface pShifterSolenoid, 
			XboxController pController, 
			Joystick pSecondaryStick,
			NetworkTable pJetsonTable,
			GearGrabberThreaded pGearGrabberThreaded) {
		
		mGearGrabber = pGearGrabberThreaded;
		
		mTimeStartedForward = -1;
		mTimeStartedBackward = -1;
		mTimeStoppedTurning = -1;
		
		mSwerveDrive = new SwerveDrive(pWheels);
		mWheels = pWheels;
		mRobotAngle = pRobotAngle;
		
		mController = pController;
		mSecondaryStick = pSecondaryStick;
		
		mShifterSolenoid = pShifterSolenoid;
		
		//initialize gyro PID
		mGyroPID_Output = new GenericPIDOutput();
		if (DriveConstants.INITIAL_SPEED_FAST) {
			mGyroPID = new LocalPIDController(
					DriveConstants.ANGLE_PID_FAST_P, DriveConstants.ANGLE_PID_FAST_I, DriveConstants.ANGLE_PID_FAST_D, mRobotAngle, mGyroPID_Output);
			
			mGyroPID.setOutputRange(-DriveConstants.ANGLE_PID_FAST_MAX, DriveConstants.ANGLE_PID_FAST_MAX);
			mGyroPID.setIZone(DriveConstants.ANGLE_PID_FAST_IZONE);
		}
		else {
			mGyroPID = new LocalPIDController(
					DriveConstants.ANGLE_PID_SLOW_P, DriveConstants.ANGLE_PID_SLOW_I, DriveConstants.ANGLE_PID_SLOW_D, mRobotAngle, mGyroPID_Output);
			
			mGyroPID.setOutputRange(-DriveConstants.ANGLE_PID_SLOW_MAX, DriveConstants.ANGLE_PID_SLOW_MAX);
			mGyroPID.setIZone(DriveConstants.ANGLE_PID_SLOW_IZONE);
		}
		
		mGyroPID.setContinuous(true);
		mGyroPID.setInputRange(0, 360);
		mGyroPID.setNegate(false);
		mGyroPID.setSetpoint(0);
		mGyroPID.setDeadband(DriveConstants.ANGLE_PID_DEADBAND);
		mGyroPID.enable();
		
		mLR_PIDOutput = new GenericPIDOutput();
		mLR_PIDInput = new GenericPIDInput();
		mLR_PID = new LocalPIDController(
				AlignerConstants.LR_P, AlignerConstants.LR_I, AlignerConstants.LR_D, 
				mLR_PIDInput, mLR_PIDOutput);
		mLR_PID.setInputRange(-1.0, 1.0);
		mLR_PID.setSetpoint(AlignerConstants.LR_SETPOINT);
		mLR_PID.setOutputRange(-AlignerConstants.LR_MAX, AlignerConstants.LR_MAX);
		mLR_PID.setDeadband(AlignerConstants.LR_DEADBAND);
		mLR_PID.enable();
		
		
		mAlignAngle_PIDOutput = new GenericPIDOutput();
		mAlignAngle_PIDInput = new GenericPIDInput();
		mAlignAngle_PID = new LocalPIDController(
				AlignerConstants.ANGLE_P, AlignerConstants.ANGLE_I, AlignerConstants.ANGLE_D, 
				mRobotAngle, mAlignAngle_PIDOutput);
		mAlignAngle_PID.setInputRange(0, 360);
		mAlignAngle_PID.setSetpoint(AlignerConstants.ANGLE_SETPOINT);
		mAlignAngle_PID.setOutputRange(-AlignerConstants.ANGLE_MAX, AlignerConstants.ANGLE_MAX);
		mAlignAngle_PID.setDeadband(AlignerConstants.ANGLE_DEADBAND);
		mAlignAngle_PID.setContinuous(true);
		mAlignAngle_PID.enable();
		
		
		
		mJetsonTable = pJetsonTable;
		mAlignState = AlignState.NONE;
		mFirstTimeAligned = -1;
	}

	/**
	 * Sets the drive motor outputs to 0 for all modules
	 */
	private void zeroMotors() {
		for(Wheel w : mWheels) {
			w.setSpeed(0);
		}
	}

	private enum AngularVelState {
		NUDGE (false, SpeedMode.SLOW),
		STICK (false, SpeedMode.FAST),
		TURN_TO_0 (true, SpeedMode.AMBIVALENT),
		TURN_TO_90 (true, SpeedMode.AMBIVALENT),
		TURN_TO_180 (true, SpeedMode.AMBIVALENT),
		TURN_TO_270 (true, SpeedMode.AMBIVALENT),
		
		TURN_TO_45 (true, SpeedMode.AMBIVALENT),
		TURN_TO_135 (true, SpeedMode.AMBIVALENT),
		TURN_TO_225 (true, SpeedMode.AMBIVALENT),
		TURN_TO_315 (true, SpeedMode.AMBIVALENT),
		
		TURN_TO_60 (true, SpeedMode.AMBIVALENT),
		TURN_TO_300 (true, SpeedMode.AMBIVALENT),
		
		TURN_TO_JOYSTICK_DIR (true, SpeedMode.SLOW),
		//ALIGN_PEG (true),
		NONE (false, SpeedMode.AMBIVALENT);
		
		boolean isGyroPID;
		SpeedMode speedMode;
		
		AngularVelState (boolean pIsGyroPID, SpeedMode pIsFast) {
			isGyroPID = pIsGyroPID;
			speedMode = pIsFast;
		}
	}
	
	private enum SpeedMode {
		SLOW,
		FAST,
		AMBIVALENT;
	}
	
	private enum RobotVelState {
		NUDGE (SpeedMode.SLOW),
		NUDGE_FIELD_RELATIVE (SpeedMode.FAST),
		STICK_ROBOT_RELATIVE (SpeedMode.FAST),
		STICK_FIELD_RELATIVE (SpeedMode.FAST),
		PREANGLE (SpeedMode.FAST),
		//ALIGN_PEG (SpeedMode.SLOW),
		NONE (SpeedMode.AMBIVALENT);
		
		SpeedMode speedMode;
		
		RobotVelState (SpeedMode pSpeedMode) {
			speedMode = pSpeedMode;
		}
	}
	
	public enum AlignState {
		NONE,
		SEARCHING,
		ALIGN_TO_PEG,
		FORWARD_INITIAL,
		FORWARD_TIME,
		RELEASING,
		BACKWARD,
		COMPLETED;
	}
	/*
	public enum AlignState {
		NONE,
		SEARCHING,
		ALIGN_ANGLE,
		FORWARD_TO_ARC,
		ROTATE_ABOUT_ARC,
		FORWARD,
		RELEASE,
		BACKWARD,
		COMPLETED;
	}*/
	/*
	public enum AlignState {
		NONE,
		SEARCHING,
		ALIGN_LR,
		FORWARD_DISTANCE,
		FORWARD_TIME,
		RELEASE,
		BACKWARD,
		COMPLETED;
	}*/
	
	
	public AlignState getAlignState() {
		return mAlignState;
	}
	
	private void setGyroPIDGains (boolean isFast) {
		if (isFast) {
			mGyroPID.setPID(DriveConstants.ANGLE_PID_FAST_P, DriveConstants.ANGLE_PID_FAST_I, DriveConstants.ANGLE_PID_FAST_D);
			mGyroPID.setOutputRange(-DriveConstants.ANGLE_PID_FAST_MAX, DriveConstants.ANGLE_PID_FAST_MAX);
			mGyroPID.setIZone(DriveConstants.ANGLE_PID_FAST_IZONE);
		}
		else {
			mGyroPID.setPID(DriveConstants.ANGLE_PID_SLOW_P, DriveConstants.ANGLE_PID_SLOW_I, DriveConstants.ANGLE_PID_SLOW_D);
			mGyroPID.setOutputRange(-DriveConstants.ANGLE_PID_SLOW_MAX, DriveConstants.ANGLE_PID_SLOW_MAX);
			mGyroPID.setIZone(DriveConstants.ANGLE_PID_SLOW_IZONE);
		}
	}
	
	public boolean jetsonSeesTarget() {
		return mJetsonTable.getBoolean(AlignerConstants.IS_VALID_KEY, false); //default to not seeing the target
	}
	
	/**
	 * determines LR offset of the target by averaging corner x-values
	 * @return LR offset of vision targets
	 */
	public double jetsonLR() {
		//return average X position
		double ul_x, ur_x, ll_x, lr_x;
		ul_x = mJetsonTable.getNumber(AlignerConstants.UL_X_KEY, AlignerConstants.LR_SETPOINT);
		ur_x = mJetsonTable.getNumber(AlignerConstants.UR_X_KEY, AlignerConstants.LR_SETPOINT);
		ll_x = mJetsonTable.getNumber(AlignerConstants.LL_X_KEY, AlignerConstants.LR_SETPOINT);
		lr_x = mJetsonTable.getNumber(AlignerConstants.LR_X_KEY, AlignerConstants.LR_SETPOINT);
		
		double average = ((ul_x + ur_x + ll_x + lr_x) / 4.0);
		return average;
	}
	
	/**
	 * determines value with the correct sign monotonic to angle offset of the target by subtracting the left height from the right height
	 * @return proxy for angle offset
	 */
	public double jetsonAngle() {
		//return left height - right height
		double ul_y, ur_y, ll_y, lr_y;
		ul_y = mJetsonTable.getNumber(AlignerConstants.UL_Y_KEY, 0.5);
		ur_y = mJetsonTable.getNumber(AlignerConstants.UR_Y_KEY, 0.5);
		ll_y = mJetsonTable.getNumber(AlignerConstants.LL_Y_KEY, 0.5);
		lr_y = mJetsonTable.getNumber(AlignerConstants.LR_Y_KEY, 0.5);
	
		double leftHeight = ll_y - ul_y;
		double rightHeight = lr_y - ur_y;
		return rightHeight - leftHeight;
	}
	
	
	/**
	 * determines value monotonic to the distance away by reciprocal of width
	 * @return proxy for distance
	 */
	public double jetsonDistance() {
		//return average X position
		double ul_x, ur_x, ll_x, lr_x;
		ul_x = mJetsonTable.getNumber(AlignerConstants.UL_X_KEY, 0.0);
		ur_x = mJetsonTable.getNumber(AlignerConstants.UR_X_KEY, 1.0);
		ll_x = mJetsonTable.getNumber(AlignerConstants.LL_X_KEY, 0.0);
		lr_x = mJetsonTable.getNumber(AlignerConstants.LR_X_KEY, 1.0);
		
		double leftX = (ul_x + ll_x) / 2;
		double rightX = (ur_x + lr_x) / 2;
		return 1.0 / (rightX - leftX);
	}
	
	
	public void align() {
		setShiftMode(false);
		/*
		if (mSecondaryStick.getRawButton(DriveConstants.LEFT_STATION_BUTTON)) AlignerConstants.ANGLE_SETPOINT = 60;
		else if (mSecondaryStick.getRawButton(DriveConstants.RIGHT_STATION_BUTTON)) AlignerConstants.ANGLE_SETPOINT = 300;
		else if (mSecondaryStick.getRawButton(DriveConstants.MIDDLE_STATION_BUTTON)) AlignerConstants.ANGLE_SETPOINT = 0;
		*/
		AlignerConstants.ANGLE_SETPOINT = 0;

		mAlignAngle_PID.setSetpoint(AlignerConstants.ANGLE_SETPOINT);
		
		boolean validTarget1 = jetsonSeesTarget();
		double lr = jetsonLR();
		double robotAngle = mRobotAngle.getAngleDegrees();
		double distance = jetsonDistance();
		boolean shouldDriveBecauseNudge = getAllWheelsInRange();
		boolean validTarget2 = jetsonSeesTarget();
		
		//has to be alid before AND after
		boolean validTarget = validTarget1 && validTarget2;
		
		Vector moveVector;
		
		SmartDashboard.putBoolean("Valid Target:", validTarget);
		SmartDashboard.putNumber("Target LR:", lr);
		SmartDashboard.putNumber("Target Distance:", distance);
		
		if (!validTarget) {
			if (mAlignState == AlignState.ALIGN_TO_PEG) mAlignState = AlignState.SEARCHING;
			if (mAlignState == AlignState.FORWARD_INITIAL) mAlignState = AlignState.FORWARD_TIME;
		}
		
		if (mAlignState == AlignState.NONE) {
			mAlignState = AlignState.SEARCHING;
			mLR_PIDInput.setVal(AlignerConstants.LR_SETPOINT);
			mAlignAngle_PIDInput.setVal(AlignerConstants.ANGLE_SETPOINT);
		}
		
		else if (mAlignState == AlignState.SEARCHING) {
			if (validTarget) {
				mAlignState = AlignState.ALIGN_TO_PEG;
			}
			else {
				mLR_PIDInput.setVal(AlignerConstants.LR_SETPOINT);
				mAlignAngle_PIDInput.setVal(AlignerConstants.ANGLE_SETPOINT);
			}
		}
		
		else if (mAlignState == AlignState.ALIGN_TO_PEG) {
			boolean lrAligned = Math.abs(lr - AlignerConstants.LR_SETPOINT) < AlignerConstants.LR_TOLERANCE;
			boolean distanceAligned = distance < AlignerConstants.DISTANCE_SETPOINT_INITIAL;
			boolean angleAligned = Math.abs(ResourceFunctions.continuousAngleDif(robotAngle, AlignerConstants.ANGLE_SETPOINT)) < AlignerConstants.ANGLE_TOLERANCE;
			boolean isAligned = lrAligned && distanceAligned && angleAligned;
			boolean isPersistentlyAligned = false;
			
			mLR_PIDInput.setVal(lr);
			mAlignAngle_PIDInput.setVal(robotAngle);
			
			if (isAligned) {
				if (mFirstTimeAligned == -1) {
					mFirstTimeAligned = System.currentTimeMillis();
				}
				else {
					long timeDif = System.currentTimeMillis() - mFirstTimeAligned;
					if (timeDif > AlignerConstants.MIN_TIME_ALIGNED) {
						isPersistentlyAligned = true;
					}
				}
			}
			else {
				mFirstTimeAligned = -1;
			}
			
			SmartDashboard.putNumber("Vision's Robot Angle:", robotAngle);
			SmartDashboard.putNumber("Vision's Angle Setpoint:", AlignerConstants.ANGLE_SETPOINT);
			SmartDashboard.putBoolean("Aligned Vision Angle", angleAligned);
			SmartDashboard.putBoolean("Aligned Vision LR", lrAligned);
			SmartDashboard.putBoolean("Aligned Vision Distance", distanceAligned);
			
			SmartDashboard.putBoolean("Aligned Vision", isAligned);
			SmartDashboard.putBoolean("Persistently Aligned Vision", isPersistentlyAligned);
			
			if (isPersistentlyAligned) {
				//mAlignState = AlignState.ALIGN_TO_PEG;
				mTimeStartedForward = -1;
				mAlignState = AlignState.FORWARD_TIME;
			}
			else {
				double forwardBackwardVel = distance > AlignerConstants.DISTANCE_SETPOINT_INITIAL ? 
						AlignerConstants.FORWARD_VEL_INITIAL : 0.0;
				
				double lrVel = 0, angularVel = 0;
				if (AlignerConstants.VISION_USE_PIDS) {
					lrVel = mLR_PIDOutput.getVal();
					angularVel = mAlignAngle_PIDOutput.getVal();
				}
				else {
					if (lrAligned) {
						lrVel = 0;
					}
					else {
						lrVel = mLR_PIDOutput.getVal() > 0 ? AlignerConstants.LR_MIN : -AlignerConstants.LR_MIN;
					}
					
					if (angleAligned) {
						angularVel = 0;
					}
					else {
						angularVel = mAlignAngle_PIDOutput.getVal() > 0 ? 
								AlignerConstants.ANGLE_MIN : -AlignerConstants.ANGLE_MIN;
					}
				}
				

				
				if (mAlignAngle_PID.GetError() < 5) {
					angularVel = 0;
				}
				if (Math.abs(lr - AlignerConstants.LR_SETPOINT) < AlignerConstants.LR_TOLERANCE/2) {
					lrVel = 0;
				}
				
				moveVector = new Vector(forwardBackwardVel, lrVel);
				
				//move relative to the peg-plane
				double angleOff = robotAngle - AlignerConstants.ANGLE_SETPOINT;
				moveVector.setAngle(moveVector.getAngle() - angleOff);
				
				SmartDashboard.putNumber("Angle PID Error:", mAlignAngle_PID.GetError());
				SmartDashboard.putNumber("Output LR:", lrVel);
				SmartDashboard.putNumber("Output Angle:", angularVel);
				SmartDashboard.putNumber("Angle Error:", mAlignAngle_PID.GetError());
				SmartDashboard.putNumber("Output Forward/Backward:", forwardBackwardVel);
				
				//calculate swerve output
				//mSwerveDrive.calculateHoldDirection(angularVel, moveVector);
				//mSwerveDrive.calculateHoldDirection(0, moveVector);
				boolean shouldDrive = true;
				if (moveVector.getMagnitude() > 0.05) {
					mSwerveDrive.calculateHoldDirection(angularVel, moveVector);
				}
				else if (angularVel > 0.05){
					mSwerveDrive.calculate(angularVel, new Vector());
				}
				else {
					mSwerveDrive.calculate(0, Vector.createPolar(angleOff, 1.0));
					shouldDrive = false;
				}
	
				
				for (int i = 0; i < 4; i++) {
					if (AlignerConstants.ACTUALLY_RUN_VISION) {
						double speed = shouldDrive ? mSwerveDrive.getOutput(i).getMagnitude() : 0.0;
						mWheels[i].set(mSwerveDrive.getOutput(i).getAngle(), speed);
					}
				}
			}
		}
		
		else if (mAlignState == AlignState.FORWARD_TIME) {
			mTimeStartedBackward = -1;
			if (mTimeStartedForward == -1) {
				mTimeStartedForward = System.currentTimeMillis();
			}
			
			if (System.currentTimeMillis() - mTimeStartedForward > AlignerConstants.TIME_FORWARD) {
				mAlignState = AlignState.RELEASING;
			}
			
			else {
				moveVector = new Vector(AlignerConstants.FORWARD_VEL_FINAL, 0);

				mSwerveDrive.calculate(0, moveVector);
				
				for (int i = 0; i < 4; i++) {
					double speed = mSwerveDrive.getOutput(i).getMagnitude();
					if (AlignerConstants.SHOULD_NUDGE_FORWARD) {
						if (!shouldDriveBecauseNudge) {
							speed = 0;
							mTimeStartedForward = -1;
						}
						mWheels[i].set(mSwerveDrive.getOutput(i).getAngle(), speed);
					}
					else {
						mWheels[i].set(mSwerveDrive.getOutput(i));
					}
					
				}
				
			}
		}
		
		else if (mAlignState == AlignState.RELEASING) {
			if (!mGearGrabber.getGearGrabbed()) mAlignState = AlignState.BACKWARD;
			else mGearGrabber.setReleaseGear(true);
		}
		
		else if (mAlignState == AlignState.BACKWARD) {
			if (mTimeStartedBackward == -1) {
				mTimeStartedBackward = System.currentTimeMillis();
			}
			if (System.currentTimeMillis() - mTimeStartedBackward > AlignerConstants.TIME_BACKWARD) {
				mAlignState = AlignState.COMPLETED;
			}
			else {

				moveVector = new Vector(-AlignerConstants.FORWARD_VEL_FINAL, 0);
				mSwerveDrive.calculateHoldDirection(0, moveVector);
				for (int i = 0; i < 4; i++) {
					if (AlignerConstants.ACTUALLY_RUN_VISION) {
						if (AlignerConstants.SHOULD_NUDGE_FORWARD) {
							double speed = shouldDriveBecauseNudge ? mSwerveDrive.getOutput(i).getMagnitude() : 0;
							mWheels[i].set(mSwerveDrive.getOutput(i).getAngle(), speed);
						}
						else {
							mWheels[i].set(mSwerveDrive.getOutput(i));
						}
					}
				}
			}
		}
		
		else if (mAlignState == AlignState.COMPLETED) {
			
		}
		
	}
	
	
	private boolean getAllWheelsInRange() {
		boolean allInRange = true;
		for (int i = 0; i < 4; i++) {
			if (!mWheels[i].isInRangeNudge()) allInRange = false;
			SmartDashboard.putBoolean("In Range Nudge #" + i, mWheels[i].isInRangeNudge());
		}
		
		return allInRange;
	}
	
	private void setShiftMode (boolean isFast) {
		if (RunConstants.SHIFTY) for (Wheel w : mWheels) w.setSpeedMode(isFast);
		if (RunConstants.SHIFTY) {
			mShifterSolenoid.set(isFast ? DriveConstants.FAST_SHIFT_DIR : DriveConstants.SLOW_SHIFT_DIR);
		}
		SmartDashboard.putBoolean("Is Fast:", isFast);
	}
	
	/**
	 * Calculate correct swerve drive movements
	 * based on XboxController and vision
	 */
	//@SuppressWarnings("unused")
	public void drive() {
		
		if (mSecondaryStick.getRawButton(DriveConstants.LEFT_STATION_BUTTON)) AlignerConstants.ANGLE_SETPOINT = 60;
		else if (mSecondaryStick.getRawButton(DriveConstants.RIGHT_STATION_BUTTON)) AlignerConstants.ANGLE_SETPOINT = 300;
		else if (mSecondaryStick.getRawButton(DriveConstants.MIDDLE_STATION_BUTTON)) AlignerConstants.ANGLE_SETPOINT = 0;
		
		
		mAlignState = AlignState.NONE;
		double curAngle = mRobotAngle.getAngleDegrees();	
		
		DriveConstants.POV_MODE_DRIVING = true; //mController.getStickButton(Hand.kLeft);
		
		//retrieve states based on joystick
		AngularVelState angularState = getAngleState();
		RobotVelState velState = getRobotVelState();
		
		//only preangle if we are doing nothing else
		if (angularState != AngularVelState.NONE && velState == RobotVelState.PREANGLE) {
			velState = RobotVelState.NONE;
		}
		
		boolean isFast = mShifterSolenoid.get() == DriveConstants.FAST_SHIFT_DIR;
		
		if (velState.speedMode == SpeedMode.SLOW || angularState.speedMode == SpeedMode.SLOW) {
			isFast = false;
		}
		else if (velState.speedMode == SpeedMode.FAST || angularState.speedMode == SpeedMode.FAST) {
			isFast = true;
		}
		
		setShiftMode(isFast);
		
		setGyroPIDGains(isFast);
		
		//initialize angulerVel & driveVec to 0
		double angularVel = 0;
		Vector driveVec = new Vector();
		
		SmartDashboard.putString("Angular state:", angularState.name());
		SmartDashboard.putString("Vel state:", velState.name());
		
		//calculate the angular velocity based on our state
		switch (angularState) {
		case NUDGE:
			angularVel = nudgeTurn();
			break;
		case STICK:
			angularVel = angularVelStick();
			break;
		case TURN_TO_0:
			angularVel = getAngularPIDVel(0, curAngle);
			break;
		case TURN_TO_90:
			angularVel = getAngularPIDVel(90, curAngle);
			break;
		case TURN_TO_180:
			angularVel = getAngularPIDVel(180, curAngle);
			break;
		case TURN_TO_270:
			angularVel = getAngularPIDVel(270, curAngle);
			break;
		case TURN_TO_45:
			angularVel = getAngularPIDVel(45, curAngle);
			break;
		case TURN_TO_135:
			angularVel = getAngularPIDVel(135, curAngle);
			break;
		case TURN_TO_225:
			angularVel = getAngularPIDVel(225, curAngle);
			break;
		case TURN_TO_315:
			angularVel = getAngularPIDVel(315, curAngle);
			break;
		case TURN_TO_60:
			angularVel = getAngularPIDVel(60, curAngle);
			break;
		case TURN_TO_300:
			angularVel = getAngularPIDVel(300, curAngle);
			break;
		case TURN_TO_JOYSTICK_DIR:
			angularVel = getAngularPIDVel(getStickAngle(Hand.kRight), curAngle);
			break;
		case NONE:
			angularVel = 0;
			break;
		}
		
		//calculate linear velocity based on current state
		switch (velState) {
		case NUDGE:
			driveVec = nudgeMove();
			break;
		case STICK_FIELD_RELATIVE:
			driveVec = swerveDriveVectorFieldRelative();
			break;
		case STICK_ROBOT_RELATIVE:
			driveVec = swerveDriveVectorRobotRelative();
			break;
		default:
			driveVec = new Vector(); //0 vector if doing nothing
		}
		
		SmartDashboard.putNumber("Angular Vel:", angularVel);
		
		//logic to determine if we need wheels to be accurate
		boolean shouldNudge = false;
		//if(velState == RobotVelState.NUDGE || velState == RobotVelState.ALIGN_PEG || angularState == AngularVelState.ALIGN_PEG) shouldNudge = true;
		if (velState == RobotVelState.NUDGE) shouldNudge = true;
		else if  (angularState == AngularVelState.NUDGE && velState == RobotVelState.NONE) shouldNudge = true;
		else shouldNudge = false;
		
		
		boolean shouldDriveWhenNudging = getAllWheelsInRange();
		SmartDashboard.putBoolean("Should drive when nudging:", shouldDriveWhenNudging);
		
		boolean canDriftCompensate = Math.abs(mRobotAngle.getAngularVelocity()) < DriveConstants.MAX_ANGULAR_VELOCITY_COMPENSATE;
		//if turning recently, dont compensate
		if (mTimeStoppedTurning > 0 && System.currentTimeMillis() - mTimeStoppedTurning < DriveConstants.TIME_AFTER_TURNING_ACTIVATE_MILLIS) { 
			canDriftCompensate = false;
		}
		
		SmartDashboard.putNumber("Time stopped turning:", mTimeStoppedTurning);
		SmartDashboard.putBoolean("Can drift compensate:", canDriftCompensate);
		
		
		//manually set the wheels to the right angle, and don't drive them; for choosing angle & then driving
		if(velState == RobotVelState.PREANGLE) {
			zeroMotors();
			double angle = getStickAngle(Hand.kLeft);
			if (!DriveConstants.POV_MODE_DRIVING) angle -= curAngle;
			for(Wheel w : mWheels) {
				w.set(angle, 0);
			}
			SmartDashboard.putBoolean("Is Drift Compensating:", false);
		}
		
		//we aren't doing anything, sit still and keep the wheel angles the same
		else if(driveVec.getMagnitude() < DriveConstants.MIN_LINEAR_VEL && 
				Math.abs(angularVel) < DriveConstants.MIN_ANGULAR_VEL &&
				!angularState.isGyroPID) {
			SmartDashboard.putBoolean("Is Drift Compensating:", false);
			zeroMotors();
		}
		
		else {
			//perform drift compensation
			if (canDriftCompensate && angularState == AngularVelState.NONE && RunConstants.DRIFT_COMPENSATION ||
					(RunConstants.TESTING_DRIFT_MODE && Math.abs(angularVel) < 0.2)) {
				SmartDashboard.putBoolean("Is Drift Compensating:", true);

				double mag = driveVec.getMagnitude();
				double gyroAngularVel = mRobotAngle.getAngularVelocity();
				double driftCompensateAngularVel = 0;
				if (mag < DriveConstants.DRIFT_COMPENSATION_ZONE_1) {
					driftCompensateAngularVel = -DriveConstants.DRIFT_COMPENSATION_P1 * gyroAngularVel;
				}
				else if (mag < DriveConstants.DRIFT_COMPENSATION_ZONE_2) {
					driftCompensateAngularVel = -DriveConstants.DRIFT_COMPENSATION_P2 * gyroAngularVel;
				}
				else {
					driftCompensateAngularVel = -DriveConstants.DRIFT_COMPENSATION_P3 * gyroAngularVel;
				}
				mSwerveDrive.calculateHoldDirection(driftCompensateAngularVel, driveVec);
				SmartDashboard.putNumber("Drift Compensation Angular Vel:", driftCompensateAngularVel);
			}

			//we are doing something active: calculate wheel vectors w/ swerve drive math
			else {
				SmartDashboard.putBoolean("Is Drift Compensating:", false);
				mSwerveDrive.calculate(angularVel, driveVec);
			}
			
			//set swerve wheels
			for(int i = 0; i < 4; i++) {
				if (shouldNudge && !shouldDriveWhenNudging) {
					mWheels[i].set(mSwerveDrive.getOutput(i).getAngle(), 0);
				}
				else {
					mWheels[i].set(mSwerveDrive.getOutput(i));
				}
				
				SmartDashboard.putNumber("Drift Output speed #" + i, mSwerveDrive.getOutput(i).getMagnitude());
			}
		}
		
		//disable the gyro PID if we didn't use it this iteration
		if (!angularState.isGyroPID) {
			mGyroPID.disable();
			mGyroPID_Output.pidWrite(Configurables.ERROR_MIN + 10); //write an error value so we know not to take the last output when reactivated
		}
		
		if (angularState != AngularVelState.NONE) mTimeStoppedTurning = System.currentTimeMillis();
	}
	
	/**
	 * Gets the angle of Xbox controller joystick 
	 * @param h Joystick to get the angle for
	 * @return Angle of the stick in degrees, with 0 degrees
	 * pointing directly up on the controller
	 */
	private double getStickAngle(Hand h) {
		double angle = Math.toDegrees(Math.atan2(-mController.getY(h), mController.getX(h)));
		angle -= 90;
		return ResourceFunctions.putAngleInRange(angle);
	}
	
	/**
	 * Gets magnitude of Xbox controller joystick 
	 * @param h Joystick to get the magnitude for
	 * @return Magnitude of the depression; 0 is not used, 1 is fully depressed
	 */
	private double getStickMag(Hand h) {
		Vector v = new Vector(mController.getX(h), mController.getY(h));
		return v.getMagnitude();
	}

	/**
	 * Determines angular velocity state,
	 * this state is then used to find the angular velocity of the swerve drive
	 * @return correct angular state
	 */
	private AngularVelState getAngleState() {
		AngularVelState angularState = AngularVelState.NONE;
		
//		if (mAlignPegState == AlignState.ALIGN_ANGLE_1 || mAlignPegState == AlignState.ALIGN_ANGLE_2) {
//			angularState = AngularVelState.ALIGN_PEG;
//		}
		if (mController.getBumper(Hand.kLeft) || mController.getBumper(Hand.kRight)) {
			angularState = AngularVelState.NUDGE;
		}
		
		//pov buttons pressed, rotate to an angle
		else if (mController.getPOV() == 0) {
			//angularState = AngularVelState.TURN_TO_0;
		}
		
		else if (mController.getPOV() == 90) {
			angularState = AngularVelState.TURN_TO_270;
		}
		
		else if (mController.getPOV() == 180) {
			//angularState = AngularVelState.TURN_TO_180;
		}
		
		else if (mController.getPOV() == 270) {
			angularState = AngularVelState.TURN_TO_90;
		}
		
//		else if (mSecondaryStick.getRawButton(3)) {
//			angularState = AngularVelState.TURN_TO_0;
//		}
		
		else if (mSecondaryStick.getRawButton(4)) {
			angularState = AngularVelState.TURN_TO_60;
		}
		
		else if (mSecondaryStick.getRawButton(5)) {
			angularState = AngularVelState.TURN_TO_300;
		}
		
		//REMOVE THIS LATER, FOR ANGULAR PID TESTING
//		else if (getStickMag(Hand.kRight) > 0.4 && mController.getAButton()) {
//			angularState = AngularVelState.TURN_TO_JOYSTICK_DIR;
//		}
		
		//if the right joystick is pushed beyond the minimum angular vel, 
		else if (Math.abs(angularVelStick()) > DriveConstants.MIN_ANGULAR_VEL_STICK) {
			angularState = AngularVelState.STICK;
		}
		
		
		return angularState;
	}
	
	/**
	 * Determines linear velocity state,
	 * this state is then used to find the correct linear velocity of the swerve drive
	 * @return correct linear velocity state
	 */
	private RobotVelState getRobotVelState() {
		RobotVelState velState = RobotVelState.NONE;
		
//		if (mAlignPegState == AlignState.ALIGN_LR) {
//			velState = RobotVelState.ALIGN_PEG;
//		}
		if(
				mController.getAButton() ||
				mController.getBButton() ||
				mController.getXButton() ||
				mController.getYButton()) {
			velState = RobotVelState.NUDGE;
			//velState = RobotVelState.NONE; //FOR ANGULAR PID TESTING
		}
		
		else if ((mController.getTriggerAxis(Hand.kRight) > DriveConstants.MIN_LINEAR_VEL || 
				 mController.getTriggerAxis(Hand.kLeft)  > DriveConstants.MIN_LINEAR_VEL) &&
				getStickMag(Hand.kLeft) > DriveConstants.MIN_DIRECTION_MAG) {
			if (DriveConstants.POV_MODE_DRIVING) velState = RobotVelState.STICK_ROBOT_RELATIVE; //set to robot relative for camera use; this needs to be worked out
			else velState = RobotVelState.STICK_FIELD_RELATIVE;
		}
		
		/*
		 * Diferent control system based on left stick field relative & right stick robot relative
		else if((mController.getTriggerAxis(Hand.kRight) > DriveConstants.MIN_LINEAR_VEL || 
				 mController.getTriggerAxis(Hand.kLeft)  > DriveConstants.MIN_LINEAR_VEL) &&
				getStickMag(Hand.kRight) > DriveConstants.MIN_DIRECTION_MAG) {
			return RobotVelState.STICK_ROBOT_RELATIVE;
		}
		*/
		
		else if(getStickMag(Hand.kLeft) > 0.3) { //this makes it easier to choose a direction and THEN start driving
			velState = RobotVelState.PREANGLE;
		}
		
		
		return velState;			
	}
	

	/**
	 * Get the direction vector for nudge driving using the letter buttons
	 * @return correct direction vector
	 */
	private Vector nudgeMove() {
		if(mController.getYButton()) {
			return Vector.createPolar(0, DriveConstants.NUDGE_MOVE_SPEED);
		}
		else if(mController.getXButton()) {
			return Vector.createPolar(90, DriveConstants.NUDGE_MOVE_SPEED);
		}
		else if(mController.getAButton()) {
			return Vector.createPolar(180, DriveConstants.NUDGE_MOVE_SPEED);
		}
		else if(mController.getBButton()) {
			return Vector.createPolar(270, DriveConstants.NUDGE_MOVE_SPEED);
		}
		
		else return new Vector();
	}
	
	/**
	 * Angular velocity using nudge bumpers
	 * @return correct angular velocity
	 */
	private double nudgeTurn() {
		if(mController.getBumper(Hand.kLeft)) return DriveConstants.NUDGE_TURN_SPEED;
		else if(mController.getBumper(Hand.kRight)) return -DriveConstants.NUDGE_TURN_SPEED;
		else return 0;
	}
	
	/**
	 * Calculates robot movement vector assuming field relative control with left xbox joystick
	 * good for controlling the robot while looking at it
	 * @return robot relative direction vector to move in a certain direction on the field
	 */
	private Vector swerveDriveVectorFieldRelative() {
		double robotAngle = mRobotAngle.getAngleDegrees();
		double driveAngle = getStickAngle(Hand.kLeft);
		driveAngle -= robotAngle; //make it field relative
		driveAngle = ResourceFunctions.putAngleInRange(driveAngle);
		
		double speed = mController.getTriggerAxis(Hand.kRight);
		speed *= Math.abs(speed); //quadratic control, finer control of lower speeds
		speed *= DriveConstants.FAST_SPEED_MULT; //lowers the maximum speed
		
		return Vector.createPolar(driveAngle, speed);
	}

	/**
	 * calculates robot movement vector assuming robot relative control with left xbox joystick
	 * good for POV control through camera
	 * @return robot relative direction vector assuming robot relative control
	 */
	private Vector swerveDriveVectorRobotRelative() {
		double robotAngle = mRobotAngle.getAngleDegrees();
		double driveAngle = getStickAngle(Hand.kLeft);
		//driveAngle -= robotAngle;
		driveAngle = ResourceFunctions.putAngleInRange(driveAngle);
		SmartDashboard.putNumber("robot angle:", robotAngle);
		SmartDashboard.putNumber("drive angle:", driveAngle);
		double speed = mController.getTriggerAxis(Hand.kRight);
		speed *= Math.abs(speed);
		speed *= DriveConstants.FAST_SPEED_MULT;
		
		return Vector.createPolar(driveAngle, speed);
	}
	
	/**
	 * Angular velocity calculated with the right joystick
	 * @return angular velocity for swerve drive
	 */
	private double angularVelStick() {
		double angularVel = mController.getX(Hand.kRight) * Math.abs(mController.getX(Hand.kRight));
		angularVel *= DriveConstants.ANGULAR_SPEED_MULT;
		angularVel = -angularVel; //correct the sign for clockwise/counter-clockwise
		return angularVel; //quadratic control for finer movements
	}
	
	/**
	 * Calculate angular velocity to turn to a certain angle
	 * @param setpointAngle angle to turn to
	 * @param curAngle current robot angle
	 * @return angular velocity required to turn to the angle
	 */
	public double getAngularPIDVel(double setpointAngle, double curAngle) {
		mGyroPID.setSetpoint(setpointAngle);
		mGyroPID.enable();
		
		//makes sure that the gyro PID has updated before we use it
		while (mGyroPID_Output.getVal() > Configurables.ERROR_MIN) {
			Timer.delay(0.005); //this is ugly, should remove; however it shoud happen very rarey
		}
		
		double vel = mGyroPID_Output.getVal();
		SmartDashboard.putNumber("Angular PID Vel", vel);
		if (Math.abs(mGyroPID.getError()) < DriveConstants.ANGLE_PID_DEADBAND) vel = 0;
		return vel;
	}

}


/*
public void align() {
	setShiftMode (false);
	
	boolean validTarget1 = jetsonSeesTarget();
	double lr = jetsonLR();
	double distance = jetsonDistance();
	boolean shouldDriveBecauseNudge = getAllWheelsInRange();
	boolean validTarget2 = jetsonSeesTarget();
	
	//has to be valid before AND after
	boolean validTarget = validTarget1 && validTarget2;
	
	Vector moveVector;
	
	SmartDashboard.putBoolean("Valid Target:", validTarget);
	SmartDashboard.putNumber("Target LR:", lr);
	SmartDashboard.putNumber("Current Time Millis:", System.currentTimeMillis());
	SmartDashboard.putNumber("Target Distance:", distance);
	SmartDashboard.putBoolean("Should Drive Because Nudge:", shouldDriveBecauseNudge);
	
	if (mAlignState != AlignState.FORWARD_TIME) mTimeStartedForward = -1;
	double lrVel = 0;
	
	switch (mAlignState) {
	case NONE:
		for (int i = 0; i < 4; i++) {
			mWheels[i].setSpeed(0);
		}
		mAlignState = AlignState.SEARCHING;
		mLR_PIDInput.setVal(AlignerConstants.LR_SETPOINT);
		mAlignAngle_PIDInput.setVal(AlignerConstants.LR_SETPOINT);
		break;
		
	case SEARCHING:
		for (int i = 0; i < 4; i++) {
			mWheels[i].setSpeed(0);
		}
		if (validTarget) {
			mAlignState = AlignState.ALIGN_LR;
			mAlignAngle_PIDInput.setVal(lr);
		}
		else {
			mAlignAngle_PIDInput.setVal(AlignerConstants.LR_SETPOINT);
		}
		break;
		
	case ALIGN_LR:
		if (!validTarget) {
			mAlignState = AlignState.SEARCHING;
			break;
		}
		mLR_PIDInput.setVal(lr);
		mLR_PID.setSetpoint(0.5);
		
		boolean isAligned = Math.abs(lr - AlignerConstants.LR_SETPOINT) < AlignerConstants.LR_TOLERANCE;
		boolean isPersistentlyAligned = false;
		
		if (isAligned) {
			if (mFirstTimeAligned == -1) {
				mFirstTimeAligned = System.currentTimeMillis();
			}
			else {
				long timeDif = System.currentTimeMillis() - mFirstTimeAligned;
				if (timeDif > AlignerConstants.MIN_TIME_ALIGNED) {
					isPersistentlyAligned = true;
				}
			}
		}
		else {
			mFirstTimeAligned = -1;
		}
		
		SmartDashboard.putBoolean("Persistently Aligned Vision", isPersistentlyAligned);
		
		if (isPersistentlyAligned) mAlignState = AlignState.FORWARD_DISTANCE;;

		lrVel = mLR_PIDOutput.getVal();
		
		SmartDashboard.putNumber("Align LR Vel:", lrVel);
		mSwerveDrive.calculate(0, new Vector (0, lrVel));
		for (int i = 0; i < 4; i++) {
			mWheels[i].set(mSwerveDrive.getOutput(i));
		}
		break;
	case FORWARD_DISTANCE:
		if (!validTarget) {
			mAlignState = AlignState.SEARCHING;
			break;
		}
		if (distance > 4) {
			for (int i = 0; i < 4; i++) {
				mWheels[i].setNudgeMode(false);
			}
			lrVel = mLR_PIDOutput.getVal();
			SmartDashboard.putNumber("Align LR Vel:", lrVel);
			mSwerveDrive.calculate(0, new Vector (AlignerConstants.FORWARD_VEL_INITIAL, lrVel));

			for (int i = 0; i < 4; i++) {
				mWheels[i].set (mSwerveDrive.getOutput(i));
			}
		}
		else {
			mAlignState = AlignState.FORWARD_TIME;
		}
		break;
	case FORWARD_TIME:
		if (mTimeStartedForward == -1) {
			mTimeStartedForward = System.currentTimeMillis();
		}
		if (System.currentTimeMillis() - mTimeStartedForward > AlignerConstants.TIME_FORWARD) {
			
		}
		break;
		
		
	default:
		for (int i = 0; i < 4; i++) {
			mWheels[i].setSpeed(0);
		}
	}
}*/

/*
public void align() {
	setShiftMode (false);
	
	boolean validTarget1 = jetsonSeesTarget();
	double lr = jetsonLR();
	double distance = jetsonDistance();
	boolean shouldDriveBecauseNudge = getAllWheelsInRange();
	boolean validTarget2 = jetsonSeesTarget();
	
	//has to be valid before AND after
	boolean validTarget = validTarget1 && validTarget2;
	
	Vector moveVector;
	
	SmartDashboard.putBoolean("Valid Target:", validTarget);
	SmartDashboard.putNumber("Target LR:", lr);
	SmartDashboard.putNumber("Current Time Millis:", System.currentTimeMillis());
	SmartDashboard.putNumber("Target Distance:", distance);
	SmartDashboard.putBoolean("Should Drive Because Nudge:", shouldDriveBecauseNudge);
	
	
	switch (mAlignState) {
	case NONE:
		for (int i = 0; i < 4; i++) {
			mWheels[i].setSpeed(0);
		}
		mAlignState = AlignState.SEARCHING;
		mLR_PIDInput.setVal(AlignerConstants.LR_SETPOINT);
		mAlignAngle_PIDInput.setVal(AlignerConstants.LR_SETPOINT);
		break;
	case SEARCHING:
		for (int i = 0; i < 4; i++) {
			mWheels[i].setSpeed(0);
		}
		if (validTarget) {
			mAlignState = AlignState.ALIGN_ANGLE;
			mAlignAngle_PIDInput.setVal(lr);
		}
		else {
			mAlignAngle_PIDInput.setVal(AlignerConstants.LR_SETPOINT);
		}
		break;
	case ALIGN_ANGLE:
		if (!validTarget) {
			mAlignState = AlignState.SEARCHING;
			break;
		}
		
		mAlignAngle_PID.setSetpoint(0.5);
		mAlignAngle_PIDInput.setVal(lr);
		
		boolean isAligned = Math.abs(lr - AlignerConstants.LR_SETPOINT) < AlignerConstants.LR_TOLERANCE;
		boolean isPersistentlyAligned = false;
		
		if (isAligned) {
			if (mFirstTimeAligned == -1) {
				mFirstTimeAligned = System.currentTimeMillis();
			}
			else {
				long timeDif = System.currentTimeMillis() - mFirstTimeAligned;
				if (timeDif > AlignerConstants.MIN_TIME_ALIGNED) {
					isPersistentlyAligned = true;
				}
			}
		}
		else {
			mFirstTimeAligned = -1;
		}
		
		SmartDashboard.putBoolean("Persistently Aligned Vision", isPersistentlyAligned);
		
		if (isPersistentlyAligned) mAlignState = AlignState.FORWARD;

		double angularVel = mAlignAngle_PIDOutput.getVal();
		SmartDashboard.putNumber("Align Angular Vel:", angularVel);
		mSwerveDrive.calculate(angularVel, new Vector());
		for (int i = 0; i < 4; i++) {
			mWheels[i].set(mSwerveDrive.getOutput(i));
		}
		break;
	case FORWARD:
		if (distance > 4) {
			for (int i = 0; i < 4; i++) {
				mWheels[i].setNudgeMode(true);
			}
			
			mSwerveDrive.calculate(0, new Vector (AlignerConstants.FORWARD_VEL_INITIAL, 0));
			
			if (shouldDriveBecauseNudge) {
				for (int i = 0; i < 4; i++) {
					mWheels[i].set (mSwerveDrive.getOutput(i));
				}
			}
			else {
				for (int i = 0; i < 4; i++) {
					mWheels[i].set(0, 0);
				}
			}
		}
		else {
			for (int i = 0; i < 4; i++) {
				mWheels[i].set(0, 0);
			}
		}
		break;
	default:
		for (int i = 0; i < 4; i++) {
			mWheels[i].set(new Vector());
		}
	}
		
}
*/

