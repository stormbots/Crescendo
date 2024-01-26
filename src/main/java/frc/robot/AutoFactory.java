// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.ChassisConstants.AutoConstants;
import frc.robot.ChassisConstants.DriveConstants;
import frc.robot.ChassisConstants.ModuleConstants;

/** Add your docs here. */
public class AutoFactory {
    RobotContainer rc;
    AutoBuilder autoBuilder;
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    ProfiledPIDController thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0,
        AutoConstants.kThetaControllerConstraints);
    
    Trajectory oneMeterForwardTraj = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(),
        new Pose2d(1,0, new Rotation2d(0)),
        trajectoryConfig);
        
    Trajectory oneMeterLeftTraj = TrajectoryGenerator.generateTrajectory(
        new Pose2d(1, 0, new Rotation2d(0)),
        List.of(),
        new Pose2d(1,1, new Rotation2d(0)),
        trajectoryConfig);
  
    public AutoFactory(RobotContainer rc){
        this.rc = rc;

        trajectoryConfig.setKinematics(rc.swerveDriveKinematics);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        Supplier<ChassisSpeeds> getSpeed = ()->{
            return rc.swerveDriveKinematics.toChassisSpeeds(rc.chassis.getWheelStates());
        };

        Consumer<ChassisSpeeds> drive = (chassisSpeeds)->{
            //is field relative? + no idea how this works as max should be 1, will check later.
            rc.chassis.drive(
                chassisSpeeds.vxMetersPerSecond/AutoConstants.kMaxSpeedMetersPerSecond, 
                chassisSpeeds.vxMetersPerSecond/AutoConstants.kMaxSpeedMetersPerSecond, 
                chassisSpeeds.omegaRadiansPerSecond/AutoConstants.kMaxAngularSpeedRadiansPerSecond, 
                false, false);
        };

        HolonomicPathFollowerConfig holonomicPathFollowerConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(ModuleConstants.kDrivingP), 
            new PIDConstants(ModuleConstants.kTurningP), 
            AutoConstants.kMaxSpeedMetersPerSecond, 
            Math.sqrt(Math.pow(DriveConstants.kTrackWidth/2, 2) + Math.pow(DriveConstants.kWheelBase/2, 2)),
            new ReplanningConfig(false, false));

        BooleanSupplier shouldFlipPath = () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        };

        AutoBuilder.configureHolonomic(
            rc.swerveDrivePoseEstimator::getEstimatedPosition, 
            rc.chassis::resetPose, 
            getSpeed, 
            drive,
            holonomicPathFollowerConfig,
            shouldFlipPath,  
            rc.chassis);
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

    public SwerveControllerCommand getSwerveControllerCommand(){
        return generateTrajectory(oneMeterForwardTraj);
    }

    public Trajectory getTrajectoryTo(Pose2d pose){
        return TrajectoryGenerator.generateTrajectory(
          rc.chassis.getPose(),
          List.of(),
          pose,
          trajectoryConfig
          );
    }
}
