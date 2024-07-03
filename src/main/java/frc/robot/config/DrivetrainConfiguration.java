// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.PIDController;


/** Add your docs here. */
public class DrivetrainConfiguration {
    public double trackWidth;
    public double wheelBase;
    public double wheelDiameterMeters;
    public int numModules;

    public class SwerveModuleConfiguration {
        //TODO: Finish this
        public int drivingCanId;
        public int turningCanId;
        public SparkPIDController drivingPIDController;
        public SparkPIDController turningController;
        public double angularOffset;
        public double DriveFreeSpeedRPM;
        public double DrivingMotorPinionTeeth;
        public double DrivingMotorSpurGearTeeth;
        
      }
     
}
