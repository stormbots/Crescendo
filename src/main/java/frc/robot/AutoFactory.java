// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.ChassisConstants.AutoConstants;

/** Add your docs here. */
public class AutoFactory {
    RobotContainer rc;
    AutoBuilder autoBuilder;
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0,
        AutoConstants.kThetaControllerConstraints);

    public AutoFactory(RobotContainer rc){
        this.rc = rc;

        //im too lazy to do this for now so i give up
        AutoBuilder.configureHolonomic(
            rc.swerveDrivePoseEstimator::getEstimatedPosition, 
            rc.chassis::resetPose, 
            rc.swerveDrivePoseEstimator, 
            (chassisSpeeds)->{
                    //is field relative? + no idea how this works as max should be 1, will check later.
                    rc.chassis.drive(
                        chassisSpeeds.vxMetersPerSecond/AutoConstants.kMaxSpeedMetersPerSecond, 
                        chassisSpeeds.vxMetersPerSecond/AutoConstants.kMaxSpeedMetersPerSecond, 
                        chassisSpeeds.omegaRadiansPerSecond/AutoConstants.kMaxAngularSpeedRadiansPerSecond, 
                        true, false);}, 
            null, 
            () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                }, 
            rc.chassis);

            trajectoryConfig.setKinematics(rc.swerveDriveKinematics);

            thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    SwerveControllerCommand generateTrajectory(Trajectory trajectory){

        // Example Trajectory
        // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        //   new Pose2d(0, 0, new Rotation2d(0)),
        //   List.of(),
        //   new Pose2d(1,0, new Rotation2d(0)),
        //   config);
    
        SwerveControllerCommand swerveControllerCommand = new
        SwerveControllerCommand(
          trajectory,
          rc.chassis::getPose, // Functional interface to feed supplier
          rc.swerveDriveKinematics,
          // Position controllers
          new PIDController(AutoConstants.kPXController, 0, 0),
          new PIDController(AutoConstants.kPYController, 0, 0),
          thetaController,
          rc.chassis::setModuleStates,
          rc.chassis
        );
    
        return swerveControllerCommand;
    
    }

    public Command ExampleAuto(){
        return new InstantCommand(()->{},rc.chassis,rc.climber);
    }

}
