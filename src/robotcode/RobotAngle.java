 package robotcode;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.PIDSource;
import sensors.MyGyroscope;
import sensors.RotationInputter;

public class RobotAngle extends RotationInputter implements PIDSource {
	AHRS mNavX;
	MyGyroscope mCustomAnalogGyro;
	AnalogGyro mWPIAnalogGyro;
	boolean mReversed;
	boolean mUsingNavx = true;
	boolean mUsingCustom = true;
	
	public RobotAngle (AHRS pNavX, boolean pReversed, double pOffset) {
		super (pReversed, pOffset);
		mReversed = pReversed;
		mNavX = pNavX;
		mCustomAnalogGyro = null;
		mWPIAnalogGyro = null;
		mUsingNavx = true;
	}
	
	public RobotAngle (MyGyroscope pAnalogGyro, boolean pReversed, double pOffset) {
		super (pReversed, pOffset);
		mReversed = pReversed;
		mNavX = null;
		mCustomAnalogGyro = pAnalogGyro;
		mUsingNavx = false;
		mUsingCustom = true;
	}
	
	public RobotAngle (AnalogGyro pAnalogGyro, boolean pReversed, double pOffset) {
		super (pReversed, pOffset);
		mReversed = pReversed;
		mNavX = null;
		mWPIAnalogGyro = pAnalogGyro;
		mUsingNavx = false;
		mUsingCustom = false;
	}
	
	public double getRawAngleDegrees() {
		if (mUsingNavx) {
			return mNavX.getAngle();
		}
		else {
			if (mUsingCustom) return mCustomAnalogGyro.getAngle();
			else return mWPIAnalogGyro.getAngle();
		}
	}
	
	public double getAngularVelocity() {
		if (mUsingNavx) {
			return (mReversed ? -1 : 1) * mNavX.getRate();
		}
		else {
			if (mUsingCustom) return (mReversed ? -1 : 1) * mCustomAnalogGyro.getRate();
			else return (mReversed ? -1 : 1) * mWPIAnalogGyro.getRate();
		}
	}

	public double pidGet() {
		return getAngleDegrees();
	}	
	
	public void reset() {
		if (mUsingNavx) {
			mNavX.reset();
		}
		else {
			if (mUsingCustom) mCustomAnalogGyro.reset();
			else mWPIAnalogGyro.reset();
			//mCustomAnalogGyro.reset();
		}
	}
}
