// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import frc.robot.Robot;
import frc.robot.config.MacAddressUtil;
import frc.robot.Constants;

import static frc.robot.config.MacAddressUtil.*;

public enum RobotIdentity {
    CONNIE,
    Practice,
    Armstrong,
    Tabi,
    Simulation;

    public static RobotIdentity getIdentity() {
        if (Robot.isReal()) {
            String mac = getMACAddress();
            if (!mac.equals("")) {
                if (mac.equals(MacAddressUtil.CurrentCompMac)) {
                    return CONNIE;
                } else if (mac.equals(MacAddressUtil.PracticeMac)) {
                    return Practice;
                } else if (mac.equals(MacAddressUtil.PastCompMac)) {
                    return Armstrong;
                } else if (mac.equals(MacAddressUtil.TabiMac)) {
                    return Armstrong;
                }
            }
            return CONNIE;
            
        } else {
            return Simulation;
        }
    }
}
