package sensors;

import edu.wpi.first.wpilibj.DigitalInput;

public class RealDigitalInput implements BooleanInputter {
	DigitalInput mDigitalInput;
	
	public RealDigitalInput (int port) {
		mDigitalInput = new DigitalInput (port);
	}
	
	@Override
	public boolean get() {
		return mDigitalInput.get();
	}

}
