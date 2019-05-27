package sensors;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import resource.ResourceFunctions;

public abstract class RotationInputter implements PIDSource  {

	private double mOffsetDegrees;
    private boolean mAdd180;
    private boolean mReversed;
    
    public RotationInputter (boolean pReversed, double pOffset) {
    	mReversed = pReversed;
    	mOffsetDegrees = pOffset;
    }
	
	public void setAdd180(boolean add) {
		mAdd180 = add;
	}
	
	protected abstract double getRawAngleDegrees();
	
	public double getAngleDegrees()
    {
		double angle = getRawAngleDegrees() * (mReversed ? -1 : 1);
		angle -= mOffsetDegrees;
		if (mAdd180) angle += 180; //when we reverse the direction, update the angle
		angle = ResourceFunctions.putAngleInRange(angle);
   	 
   	 	return angle;
    }
    
    
    public double pidGet()
    {
        return (getAngleDegrees());
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

	//@Override
	public void setOffset(double offset) {
		mOffsetDegrees = offset;
	}
	
}
