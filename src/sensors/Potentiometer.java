package sensors;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import resource.ResourceFunctions;

public class Potentiometer {
	AnalogInput mPot;
	double mOffset, mMult;
	
	public Potentiometer(AnalogInput pPot, double pMult, double pOffset) {
		mPot = pPot;
		mOffset = pOffset;
		mMult = pMult;
	}
	
	public Potentiometer(AnalogInput pPot, double min, double max, double minEquiv, double maxEquiv) {
		mPot = pPot;
		//mult * min + off = minEquiv
		//mult * max + off = maxEquiv
		//mult(max - min) = maxEquiv - minEquiv
		//mult = (maxEquiv-minEquiv) / (max - min)
		//off = minEquiv - min * mult
		mMult = (maxEquiv - minEquiv) / (max - min);
		mOffset = (minEquiv - min * mMult);
	}



	//@Override
	public void setPIDSourceType(@SuppressWarnings("unused") PIDSourceType pidSource) {
		// TODO Auto-generated method stub
		
	}

	//@Override
	public PIDSourceType getPIDSourceType() {
		// TODO Auto-generated method stub
		return null;
	}

	//@Override
	public double pidGet() {
		return getAngleDegrees();
	}

	//@Override
	public double getAngleDegrees() {
		double raw = mPot.getVoltage();
		SmartDashboard.putNumber("Raw pot:", raw);
		double scaled = raw * mMult;
		return ResourceFunctions.putAngleInRange(scaled + mOffset);
	}


	//@Override
	public void setOffset(double offset) {
		mOffset = offset;
	}


	//@Override
	public void setMult(double mult) {
		mMult = mult;
	}
	
	
}
