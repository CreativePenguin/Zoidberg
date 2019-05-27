package constants;

public class RunConstants {
	public enum RunMode {
		TEST_TALONS,
		CRAB,
		FULL,
		WINCH,
		TEST_PNEUMATICS,

		DRIVE_CONSTANT_PERCENT,
		ROTATE_TURN_MOTORS,
		ROTATE_TIMER,

		LED_TEST,
		JETSON_TEST,

		GYRO_TEST;
	}

	public enum AngleMode {
		NavX_USB,
		NavX_I2C,
		CustomAnalogGyro,
		WPIAnalogGyro;
	}

	public static final RunMode RUN_MODE = RunMode.TEST_TALONS;
	public static boolean 
	PNEUMATICS = true, 
	SHIFTY = true, 
	SPEED_PID = true, 
	GET_FROM_SMARTDASHBOARD = false,
	DRIFT_COMPENSATION = false,
	TESTING_DRIFT_MODE = false;


	public static final AngleMode ANGLE_MODE = AngleMode.WPIAnalogGyro;
}
