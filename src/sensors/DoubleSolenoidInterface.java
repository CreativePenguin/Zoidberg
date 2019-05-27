package sensors;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public interface DoubleSolenoidInterface {
	public void set (Value v);
	public Value get();
}
