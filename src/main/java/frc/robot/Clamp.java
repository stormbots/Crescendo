package frc.robot;

public class Clamp {
    public static double clamp(double x, double min, double max) {
        if (x < min) {
            return min;
        }
        if (x > max) {
            return max;
        }
        return x;
    }

    /**
     * 
     * @param x value in question
     * @param min minimum of range
     * @param max maximum of range
     * @return true if x is inclusive in range min < X < max
     */
    public static boolean bounded(double x, double min, double max) {
        if (x < min) {
            return false;
        }
        if (x > max) {
            return false;
        }
        return true;
    }
}
