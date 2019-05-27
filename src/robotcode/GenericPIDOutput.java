package robotcode;

import edu.wpi.first.wpilibj.PIDOutput;

public class GenericPIDOutput implements PIDOutput{
	private double mVal;

	@Override
	public void pidWrite(double output) {
		mVal = output;
	}
	
	public double getVal() {
		return mVal;
	}
	
}
