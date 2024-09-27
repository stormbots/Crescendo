// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import frc.robot.config.Robots.CompBot;
import frc.robot.config.Robots.PracticeBot;
import frc.robot.config.Robots.Tabi;

/** Add your docs here. */
public interface RobotConstants {
    DrivetrainConfiguration getDrivetrainConfiguration();

    static RobotConstants getRobotConstants(RobotIdentity robot){
        switch (robot) {
            case CONNIE:
                return new CompBot();
        
            case Practice:
                return new PracticeBot();
            
            case Tabi:
                return new Tabi();
            
            default:
            //return new CompBot();
                return new Tabi();


        }

    }
    
} 
