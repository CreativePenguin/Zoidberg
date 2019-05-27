package constants;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class DriveConstants {
	//speed multipliers
	public static final double //SLOW_SPEED_MULT = 1.0, //left trigger
	FAST_SPEED_MULT = 1.0, //right trigger
	ANGULAR_SPEED_MULT = 0.3; //angular

	public static final long
	MIN_TIME_IN_RANGE_NUDGE_MILLIS = 10; //doesnt wait any time

	//speed mins, when lower than these don't do anything
	public static final double MIN_ANGULAR_VEL = 0.01; 
	public static final double MIN_ANGULAR_VEL_STICK = 0.01;
	public static final double MIN_LINEAR_VEL = 0.05;
	public static final double MIN_DIRECTION_MAG = 0.5; //refers to left joystick magnitude for choosing swerve direction

	public static boolean POV_MODE_DRIVING = false; //not final so we can change it during the game

	public static final double MAX_INDIVIDUAL_VELOCITY = 1.0; //This SHOULD be 1, reduced for testing purposes; never sets wheel speed above this

	//encoder constants
	public static final double[] ENCODER_OFFSETS = new double[]
			{49.7, 158 + 180, 299.5 + 180, 170};

	//for RPS conversion
	public static final double RAW_TO_RPS = (1.0 / 60.0) * (34.0/70.0); //(10.0 / 1024.0) * (34.0/70.0);//1.0; //34.0 / 70.0;
	public static final double MAX_RPS_FAST_MODE = 12, MAX_RPS_SLOW_MODE = 6;

	public static final boolean[] TURN_ENCODER_REVERSED = new boolean[] {false, false, false, false};
	public static final boolean[] DRIVE_REVERSED = new boolean[] {false, false, false, false};
	public static final boolean[] TURN_REVERSED = new boolean[] {false, false, false, false};
	public static final boolean[] DRIVE_ENCODER_REVERSED = new boolean[] {false, false, false, false};



	//wheel PID vals
	public static final double[] ROTATION_PID_P = new double[] {0.015, 0.015, 0.015, 0.02};
	public static final double[] ROTATION_PID_I = new double[] {0.002, 0.002, 0.002, 0.002};
	public static final double[] ROTATION_PID_D = new double[] {0.01, 0.01, 0.01, 0.01};
	//public static final double[] ROTATION_PID_IZONE = new double[] {10, 10, 10, 10};
	public static final double[] ROTATION_PID_IZONE = new double[] {15, 15, 15, 15};

	/*
	 * before townsend harris
	public static  double[] FAST_SPEED_PID_P = new double[] {0.001, 0.001, 0.001, 0.001};
	public static  double[] FAST_SPEED_PID_I = new double[] {0.000001, 0.000001, 0.000001, 0.000001};
	public static  double[] FAST_SPEED_PID_D = new double[] {0, 0, 0, 0};
	public static  double[] FAST_SPEED_PID_F = new double[] {0.001, 0.001, 0.001, 0.001};
	public static  int[]    FAST_SPEED_PID_IZONE = new int[] {0, 0, 0, 0};

	public static  double[] SLOW_SPEED_PID_P = new double[] {0.001, 0.001, 0.001, 0.001};
	public static  double[] SLOW_SPEED_PID_I = new double[] {0.000001, 0.000001, 0.000001, 0.00000};
	public static  double[] SLOW_SPEED_PID_D = new double[] {0, 0, 0, 0};
	public static  double[] SLOW_SPEED_PID_F = new double[] {0.0018, 0.0018, 0.0022, 0.0022};
	public static  int[]    SLOW_SPEED_PID_IZONE = new int[] {0, 0, 0, 0};
	 */

	public static  double[] 
			FAST_SPEED_PID_P = new double[] {0.001, 0.001, 0.001, 0.001},
			FAST_SPEED_PID_I = new double[] {0.0, 0.0, 0.0, 0.0},
			FAST_SPEED_PID_D = new double[] {0, 0, 0, 0},
			FAST_SPEED_PID_F = new double[] {0.00083, 0.00083, 0.00085, 0.00088};

	public static  double[] 
			SLOW_SPEED_PID_P = new double[] {0.001, 0.001, 0.001, 0.001},
			SLOW_SPEED_PID_I = new double[] {0.0, 0.0, 0.0, 0.0},
			SLOW_SPEED_PID_D = new double[] {0, 0, 0, 0},
			SLOW_SPEED_PID_F = new double[] {0.00145, 0.0016, 0.0016, 0.0015};

	//wheel positions
	/*
	 * NE: x = 8.5, y = -9.75
	 * NW: x = 8.5, y = 9.75
	 * SW: x = -9.5, y = 10
	 * SE: x = -9.5, y = -10
	 */
	public static final double[] X_OFF = new double[]
			{19.0/2.0, 19.0/2.0, -19.0/2.0, -19.0/2.0};

	public static final double[] Y_OFF = new double[]
			{-22.0/2.0, 22.0/2.0, 22.0/2.0, -22.0/2.0};


	public static final double NAVX_ANGLE_OFFSET = 0;
	public static final double NUDGE_ANGLE_TOLERANCE = 3;

	public static final double NUDGE_MOVE_SPEED = 0.4;
	public static final double NUDGE_TURN_SPEED = 0.4;


	//for turning robot to an angle
	public static final double 
	ANGLE_PID_SLOW_P = 0.02,
	ANGLE_PID_SLOW_I = 0.003, //0.005,
	ANGLE_PID_SLOW_D = 0.0, //0.04,
	ANGLE_PID_SLOW_IZONE = 7,
	ANGLE_PID_SLOW_MAX = 0.5,
	ANGLE_PID_SLOW_MIN = 0.0;

	public static final double 
	ANGLE_PID_FAST_P = 0.007,
	ANGLE_PID_FAST_I = 0.00, //0.005,
	ANGLE_PID_FAST_D = 0.0, //0.04,
	ANGLE_PID_FAST_IZONE = 5,
	ANGLE_PID_FAST_MAX = 0.5,
	ANGLE_PID_FAST_MIN = 0.0;

	public static final double
	ANGLE_PID_DEADBAND = 0;

	public static final double 
	MAX_ANGULAR_VELOCITY_COMPENSATE = 2,
	TIME_AFTER_TURNING_ACTIVATE_MILLIS = 500,
	DRIFT_COMPENSATION_ZONE_1 = 0.4,
	DRIFT_COMPENSATION_ZONE_2 = 0.7,
	DRIFT_COMPENSATION_P1 = 0.2,
	DRIFT_COMPENSATION_P2 = 0.5,
	DRIFT_COMPENSATION_P3 = 1.0;


	public static final Value SLOW_SHIFT_DIR = Value.kForward, FAST_SHIFT_DIR = Value.kReverse;
	public static final boolean INITIAL_SPEED_FAST = true;

	public static final int 
	LEFT_STATION_BUTTON = 4,
	RIGHT_STATION_BUTTON = 5,
	MIDDLE_STATION_BUTTON = 3;


}

