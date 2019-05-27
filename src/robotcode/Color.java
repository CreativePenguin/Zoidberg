package robotcode;

public class Color {
	private int mRed;
	private int mGreen;
	private int mBlue;
	private double mBrightness;

	public Color(int r, int g, int b, double br) {
		mRed = r;
		mGreen = g;
		mBlue = b;
		mBrightness = br;
	}

	public Color(int r, int g, int b) {
		this(r, g, b, 1);
	}

	public int getRed() {
		return mRed;
	}

	public int getGreen() {
		return mGreen;
	}

	public int getBlue() {
		return mBlue;
	}

	public double getBrightness() {
		return mBrightness;
	}
}