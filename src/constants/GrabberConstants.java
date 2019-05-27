
package constants;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class GrabberConstants {
	//pneumatic states
	public static final Value   
	FRAME_UP = Value.kReverse,
	FRAME_DOWN = Value.kForward;

	public static final boolean
	CLAMPER_OPEN = false,
	CLAMPER_CLOSED = true,
	SNATCHER_OPEN = true,
	SNATCHER_CLOSED = false;

	//break beam normally open or closed
	public static final boolean 
	GEAR_PRESENT_BREAK_BEAM = false, 
	GEAR_PRESENT_LIMIT_SWITCH = true;

	//time to activate pistons
	public static final double 
	FRAME_TIME_DOWN = 2.0, //1.0,
	FRAME_TIME_UP = 0.3,
	CLAMPER_TIME_OPEN = 0.3,
	CLAMPER_TIME_CLOSE = 0.5,
	SNATCHER_TIME_RELEASE_PRESSURE_OPEN = 0.02,
	SNATCHER_TIME_RELEASE_PRESSURE_CLOSE = 0.05,
	SNATCHER_TIME_OPEN = 0.3,
	SNATCHER_TIME_CLOSE = 0.3,
	WAIT_AFTER_ALIGNED = 0.2,
	WAIT_AFTER_BUMPSWITCH = 0.2;

	public static final double ALIGNER_TALON_SPEED = 0.5;

	public static final int MAX_TIME_FLIPPY_MILLIS = 3000;
}
