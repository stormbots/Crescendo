package frc.robot;

public class Clamp{
	public static double clamp(double x, double min, double max) {
		if(x < min) {
			return min;
		}
		if(x > max) {
			return max;
		}
		return x;
	}
	
	/**
	 * @param x Value in question
	 * @param min Minimum of range
	 * @param max Maximum of range
	 * @return	True if X is inclusive in range min < X < max
	 */
	public static boolean bounded(double x, double min, double max) {
		if(x < min) {
			return false;
		}
		if(x > max) {
			return false;
		}
		return true;
	}
}