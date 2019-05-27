package sensors;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;

public class TalonRelativeEncoder extends SpeedInputter {
	CANTalon mTalon;
	
	public TalonRelativeEncoder (CANTalon pTalon, double pTicksToRPM) {
		super(pTicksToRPM);
		mTalon = pTalon;
		
		mTalon.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
	}

	@Override
	public double getRawTicksPerSecond() {
		return mTalon.getSpeed();
	}
	
	
}
