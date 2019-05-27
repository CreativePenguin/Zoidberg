package sensors;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class RealDoubleSolenoid implements DoubleSolenoidInterface {
	DoubleSolenoid mDoubleSolenoid;
	
	public RealDoubleSolenoid (int port0, int port1) {
		mDoubleSolenoid = new DoubleSolenoid (port0, port1);
	}

	@Override
	public void set(Value v) {
		mDoubleSolenoid.set(v);
	}
	
	public Value get() {
		return mDoubleSolenoid.get();
	}
}
