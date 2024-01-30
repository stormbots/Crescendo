package com.stormbots;

public class Lerp{

    private double inMax;
    private double inMin;
    private double outputMax;
    private double outputMin;

    /**
     * Configure a range conversion instance using linear interpolation
     * @param inMin start value of your input range
     * @param inMax end value of your input range
     * @param outputMin 
     * @param outputMax
     */
    public Lerp( double inMin, double inMax, double outputMin, double outputMax) {
        this.inMax = inMax;
        this.inMin = inMin;
        this.outputMin = outputMin;
        this.outputMax = outputMax;
    }
    
    public double get(double input){
        return lerp(input, this.inMin, this.inMax, this.outputMin, this.outputMax);
    }
        
    /**
     * 
     * @param output
     * @return
     */
    public double getReverse(double output){
        return lerp( output, this.outputMin, this.outputMax, this.inMin, this.inMax);
    }


	/**
	 * Convert one linear data range into another.
     *
	 * @param input
	 * @param inMin
	 * @param inMax
	 * @param outputMin
	 * @param outputMax
	 * @return linear map function for conversion from a unit to another
	 */
	public static double lerp(double input,double inMin, double inMax, double outputMin,double outputMax){
        return (input/(inMax-inMin)-inMin/(inMax-inMin))*(outputMax-outputMin)+outputMin;
    }

    	/**
	 * Arduino Map function. <br>
	 * Prefer the {@link #lerp()} function which does the same thing, but is not improperly 
	 * named.
	 */
	@Deprecated
	public static double map(double x, double in_min, double in_max, double out_min, double out_max) {
		return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
	}
}