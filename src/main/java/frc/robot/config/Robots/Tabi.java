// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config.Robots;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import frc.robot.config.DrivetrainConfiguration;
import frc.robot.config.DrivetrainConfiguration.SwerveModuleConfiguration;
import frc.robot.config.RobotConstants;
import frc.robot.subsystems.Chassis.ChassisConstants.ModuleConstants;
import frc.robot.subsystems.Chassis.ChassisConstants.NeoMotorConstants;

/** Add your docs here. */
public class Tabi implements RobotConstants{
    double trackWidthIn = 12.0;
    double wheelBaseIn = 12.0;
    double wheelDiameterMeters = Units.inchesToMeters(3);
    int numModules = 2;
    boolean isUsingSparkMax = true;
    int frontDriveCanId = 2;
    int backDriveCanId = 1;
    int frontTurnCanId = 6;
    int backTurnCanId = 5;
    PIDController drivingPIDController = new PIDController(ModuleConstants.kDrivingP_Real, ModuleConstants.kDrivingI_Real, ModuleConstants.kDrivingD_Real);
    PIDController turningPIDController = new PIDController(ModuleConstants.kTurningP_Real, ModuleConstants.kTurningI_Real, ModuleConstants.kTurningD_Real);
    double drivingFF = ModuleConstants.kDrivingFF_Real;
    double turningFF = ModuleConstants.kTurningFF_Real;
    double frontAngularOffset = 45;
    double backAngularOffset = -135;
    double driveFreeSpeedRPM = NeoMotorConstants.kNEOFreeSpeedRpm;
    int drivingMotorPinionTeeth = 14;
    int drivingMotorSpurGearTeeth = 22;

    DrivetrainConfiguration tabi = new DrivetrainConfiguration(12.0, 12.0, Units.inchesToMeters(3), 2);
    SwerveModuleConfiguration frontRight = tabi.new SwerveModuleConfiguration(
    frontDriveCanId, frontTurnCanId, drivingPIDController, turningPIDController, drivingFF, turningFF, 
    frontAngularOffset, driveFreeSpeedRPM, drivingMotorPinionTeeth, drivingMotorSpurGearTeeth);
    SwerveModuleConfiguration backLeft = tabi.new SwerveModuleConfiguration(
    backDriveCanId, backTurnCanId, drivingPIDController, turningPIDController, drivingFF, turningFF, 
    backAngularOffset, driveFreeSpeedRPM, drivingMotorPinionTeeth, drivingMotorSpurGearTeeth);

    SwerveModuleConfiguration[] modules = tabi.listModules(frontRight,backLeft);
    double[] offsetList = tabi.listModuleOffsets(frontRight, backLeft);

    //TODO: add list of modules

    @Override
    public DrivetrainConfiguration getDrivetrainConfiguration() {
        return tabi;
    }
}

