package robotcode;


import com.ctre.CANTalon;

import constants.GrabberConstants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import sensors.BooleanInputter;
import sensors.DoubleSolenoidInterface;

public class GearGrabberThreaded implements Runnable {

	private boolean mEnabled;
	private DoubleSolenoidInterface mFrame;
	private Solenoid mClamper, mSnatcher;
	private BooleanInputter mBreakBeam, mLimitSwitch;
	
	private boolean mGearGrabbed;
	private int mFrameNum;
	
	private CANTalon mAlignerTalon;
	
	private boolean mShouldReleaseGear;
	
	private XboxController mXbox;
	//private Joystick mStick;
	
	
	public GearGrabberThreaded (BooleanInputter pBreakBeam, BooleanInputter pLimitSwitch, 
			DoubleSolenoidInterface pFrame, Solenoid pClamper, Solenoid pSnatcher,
			CANTalon pAlignerTalon, XboxController pXbox) {
		mEnabled = true;
		
		mBreakBeam = pBreakBeam;
		mLimitSwitch = pLimitSwitch;
		
		mFrame = pFrame;
		mClamper = pClamper;
		mSnatcher = pSnatcher;
		
		mGearGrabbed = false;
		
		mXbox = pXbox;
		
		mFrame.set (GrabberConstants.FRAME_UP);
		mClamper.set (GrabberConstants.CLAMPER_OPEN);
		mSnatcher.set (GrabberConstants.SNATCHER_OPEN);
		
		mAlignerTalon = pAlignerTalon;
		
		mFrameNum = 0;
	}
	
	public synchronized void setReleaseGear(boolean pReleaseGear) {
		mShouldReleaseGear = mGearGrabbed && pReleaseGear;
	}
	
	public synchronized boolean getGearGrabbed() {
		return mGearGrabbed;
	}
		
	private boolean snatchGear() {
		mAlignerTalon.set(GrabberConstants.ALIGNER_TALON_SPEED);
		
		mSnatcher.set(GrabberConstants.SNATCHER_CLOSED);
		Timer.delay(GrabberConstants.SNATCHER_TIME_CLOSE);
		
		boolean consistentlyHittingBumpswitch = false;
		while (!consistentlyHittingBumpswitch) {
			//open and close until it works
			long timeStarted = System.currentTimeMillis();
			while (mLimitSwitch.get() != GrabberConstants.GEAR_PRESENT_LIMIT_SWITCH) {
				mSnatcher.set(GrabberConstants.SNATCHER_OPEN);
				Timer.delay(GrabberConstants.SNATCHER_TIME_RELEASE_PRESSURE_OPEN);
				mSnatcher.set(GrabberConstants.SNATCHER_CLOSED);
				Timer.delay(GrabberConstants.SNATCHER_TIME_RELEASE_PRESSURE_CLOSE);
				
				if ((System.currentTimeMillis() - timeStarted) > GrabberConstants.MAX_TIME_FLIPPY_MILLIS) {
					mAlignerTalon.set(0);
					return false;
				}
			}
			
			Timer.delay (GrabberConstants.WAIT_AFTER_ALIGNED);
			if (mLimitSwitch.get() == GrabberConstants.GEAR_PRESENT_LIMIT_SWITCH) consistentlyHittingBumpswitch = true;
			else consistentlyHittingBumpswitch = false;
		}
		
		
		while (Math.abs(mXbox.getTriggerAxis(Hand.kRight)) < 0.3 && mEnabled && !Thread.interrupted()){
			Timer.delay(0.005);
		}
		
		mAlignerTalon.set(0);

		return true;
		//wait until it hits the switch
//		while (mLimitSwitch.get() != GrabberConstants.GEAR_PRESENT_BREAK_BEAM) {
//			Timer.delay(0.005);
//		}
	}
	
	private void grabGear() {
		//set initial states
		mSnatcher.set(GrabberConstants.SNATCHER_CLOSED);
		mClamper.set(GrabberConstants.CLAMPER_OPEN);
		mFrame.set(GrabberConstants.FRAME_UP);
		
		mFrame.set(GrabberConstants.FRAME_DOWN);
		Timer.delay(GrabberConstants.FRAME_TIME_DOWN);
		
		mClamper.set(GrabberConstants.CLAMPER_CLOSED);
		mSnatcher.set(GrabberConstants.SNATCHER_OPEN);
		Timer.delay(Math.max(GrabberConstants.CLAMPER_TIME_CLOSE, GrabberConstants.SNATCHER_TIME_OPEN));
		
		mFrame.set(GrabberConstants.FRAME_UP);
		Timer.delay(GrabberConstants.FRAME_TIME_UP);
	}
	
	private void releaseGear() {
		mClamper.set(GrabberConstants.CLAMPER_OPEN);
		Timer.delay(GrabberConstants.CLAMPER_TIME_OPEN);
	}
	
	private void process() {
		mFrameNum++;
		SmartDashboard.putString ("Gear grabbed", mGearGrabbed + "");
		SmartDashboard.putNumber ("Frame Num", mFrameNum);

		SmartDashboard.putString ("Frame Piston", mFrame.get().toString());
		SmartDashboard.putString ("Clamper Piston", mClamper.get() + "");
		SmartDashboard.putString ("Snatcher Piston", mSnatcher.get() + "");
		
		if (mGearGrabbed) {
			mFrame.set (GrabberConstants.FRAME_UP);
			mClamper.set (GrabberConstants.CLAMPER_CLOSED);
			mSnatcher.set (GrabberConstants.SNATCHER_OPEN);
		}
		
		else {
			mFrame.set (GrabberConstants.FRAME_UP);
			mClamper.set (GrabberConstants.CLAMPER_OPEN);
			mSnatcher.set (GrabberConstants.SNATCHER_OPEN);
		}
		
		//snatch the gear
		if (mBreakBeam.get() == GrabberConstants.GEAR_PRESENT_BREAK_BEAM && !mGearGrabbed) {
			if (snatchGear()) {
				grabGear();
				mGearGrabbed = true;
			}
		}

		if (mGearGrabbed && (mShouldReleaseGear || mXbox.getStartButton())) {
			releaseGear();
			mGearGrabbed = false;
			mShouldReleaseGear = false;
		}
		
	}
	
	public synchronized void disable() {
		mEnabled = false;
	}
	
	public synchronized void enable() {
		mEnabled = true;
	}
	
	@Override
	public void run() {
		while (mEnabled && !Thread.interrupted()) {
			process();
			Timer.delay(0.05);
		}
		mFrame.set (GrabberConstants.FRAME_UP);
		mClamper.set (GrabberConstants.CLAMPER_OPEN);
		mSnatcher.set (GrabberConstants.SNATCHER_OPEN);
		mAlignerTalon.set (0);
	}
	
}
