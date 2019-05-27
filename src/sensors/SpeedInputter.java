package sensors;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public abstract class SpeedInputter implements PIDSource {
	private double mTicksToRPS;
	
	public SpeedInputter (double pTicksToRPM) {
		mTicksToRPS = pTicksToRPM;
	}
	
	public abstract double getRawTicksPerSecond();
	
	public double getRPS() {
		return getRawTicksPerSecond() * mTicksToRPS;
	}
	
	public double pidGet() {
		return getRPS();
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
