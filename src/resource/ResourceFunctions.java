package resource;

public class ResourceFunctions {
	public static double putAngleInRange(double angle) {
		return (angle + 360000000) % 360;
	}
	
	public static double continuousAngleDif (double angle1, double angle2) {
		double dif = putAngleInRange(angle1) - putAngleInRange(angle2);
		dif = putAngleInRange(dif);
		//if (dif > 180) dif -= 180;
		if (dif > 180) dif = dif - 360;
		return dif;
	}

	public static boolean equals(double a, double b) {
		return (Math.abs(a - b) < 0.001);
	}

	public static double PutNumInAbsoluteRange(
			double val, double min, double max) {
		
		double abs = Math.abs(val);
		double newVal = 0;
		if(abs < min) {
			newVal = val < 0 ? -min : min;
		}
		else if(abs > max) {
			newVal = val < 0 ? -max : max;
		}
		else {
			newVal = val;
		}
		return newVal;
		
	}
}
