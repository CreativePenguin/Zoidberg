package sensors;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;

public class TalonAbsoluteEncoder extends RotationInputter {
	CANTalon mTalon;
	
	public TalonAbsoluteEncoder (CANTalon pTalon, boolean pReverse, double pOffset) {
		super(pReverse, pOffset);
		mTalon = pTalon;
		
		mTalon.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Absolute);
	}
	
	@Override
	protected double getRawAngleDegrees() {
		return mTalon.getPosition() * 360;
	}

}
