/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package  sensors;


import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 *
 * @author 3419
 */
public class USDigitalMA3Encoder extends RotationInputter
{

    private double mLastRawAngleDegrees;
    private Counter mDevice;
  
    
    //offset should be calibrated such that point the wheel straight forwards and
    //read that raw value;
    //Straight forwards then returns "0" as the GetAngle()

    public USDigitalMA3Encoder(int pChannel, double pOffset, boolean pReversed)
    {
    	super (pReversed, pOffset);
        mDevice = new Counter(new DigitalInput(pChannel));
        mDevice.setSemiPeriodMode(true);
        mLastRawAngleDegrees = getRawAngleDegrees();
    }
    

    public double getRawAngleDegrees()
     {
		 double period = mDevice.getPeriod();
		 if(period == 0)
		 {
		     return mLastRawAngleDegrees;
		 }
		 else
		 {
		    int x = (int) Math.round (((period * 1000.0 / 4.098) * 4098.0) - 1.0);
		    if(x > 4094)
		    {
		    	x = 4095;
		    }
		    double deviceValue = ((double)x / 4097.0) * 360.0;
		    if(deviceValue > 0.0 && deviceValue <= 360.0) //we're making zero not a valid state because sometimes it gives faulty zeroes.
		    {
		    	mLastRawAngleDegrees = deviceValue;
		    	return mLastRawAngleDegrees;
		    }
		    else
		    {
		    	return mLastRawAngleDegrees;
		    }
		 }
     }
}
    
