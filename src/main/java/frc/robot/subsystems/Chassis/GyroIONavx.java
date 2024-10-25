// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Chassis;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI.Port;

/** Add your docs here. */
public class GyroIONavx implements GyroIO {
    private final AHRS navx = new AHRS(Port.kMXP, (byte) 200);
    private final Rotation2d yaw = navx.getRotation2d();
    private final Double angularVelocityDegrees = navx.getRate();
}
