package com.stormbots;

import java.util.Arrays;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LUT {
    double[][] table;

    /** Create a look up table using a nested list
     *  Assumes key values are in increasing order
     */
    public LUT(double[][] table){
        this.table = table;
    }

    public double[] get(double key){
        //if we're smaller than our smallest key, return smallest value
        if(key < table[0][0]){ return Arrays.copyOfRange(table[0], 1, table[0].length);}

        //if we're larger than our largest value, return largest value
        if(key > table[table.length-1][0]){ return Arrays.copyOfRange(table[table.length-1], 1, table[table.length-1].length);}

        //with those out of the way, we can safely assume we're in between two points that exist.
        // Find it, split the difference
        for( var row=0; row<table.length-1; row++){

            // Check to see if the _next_ row is larger than our key value.
            // This implies the current row value is the one below it.
            if(table[row+1][0]>key){
                //Interpolate between this row and the next, and return those values
                double[] value=new double[table[row].length-1];
                for(var col=1; col<table[row].length; col++){
                    value[col-1] = Lerp.lerp(key, table[row][0], table[row+1][0], table[row][col], table[row+1][col]);
                }
                return value;
            }
        }

        //This should be unreachable, but returns the last list item 
        return Arrays.copyOfRange(table[table.length-1], 1, table[table.length-1].length);
    }

}
