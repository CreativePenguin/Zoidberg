package constants;


public class Ports {
	public static final int NE = 0, NW = 1, SW = 2, SE = 3;

	public static final int[] TURN = new int[] {0, 1, 2, 3};
	public static final int[] DRIVE = new int[] {4, 5, 6, 7};

	public static final int 
	SNATCHER_PISTON = 6,
	CLAMPER_PISTON = 7,

	FRAME_PISTON_IN = 4,
	FRAME_PISTON_OUT = 5,


	SHIFTER_SOLENOID_IN = 2, 
	SHIFTER_SOLENOID_OUT = 3;

	public static final int GEAR_ALIGNER = 10;

	public static final int BREAK_BEAM = 9, LIMIT_SWITCH = 8;
	public static final int LED_DATA = 1, LED_CLOCK = 0;

	public static final int ANALOG_GYRO_CHANNEL = 0;

	public static final int WINCH_TALON = 11;

}
