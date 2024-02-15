// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import javax.swing.text.html.ParagraphView;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.ChassisConstants.AutoConstants;

/** Add your docs here. */
public class AutoFactory {
    RobotContainer rc;

    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        Units.MetersPerSecond.of(AutoConstants.kMaxSpeedMetersPerSecond), 
        Units.MetersPerSecondPerSecond.of(AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    );

    ProfiledPIDController thetaController = new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(
        Math.PI, Math.PI));

    public AutoFactory(RobotContainer rc){
        this.rc = rc;
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public Command ExampleAuto(){
        return new InstantCommand(()->{},rc.chassis,rc.climber);
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
          //Probably percent per error (so 0.1 means 10% output for every meter)
          new PIDController(0, 0, 0),
          new PIDController(0, 0, 0),
          thetaController,
          rc.chassis::setModuleStates,
          rc.chassis
        );
    
        return swerveControllerCommand;
    
    }

    public Command getTwoMeterForwardTrajectory(){
        return generateTrajectory(
            TrajectoryGenerator.generateTrajectory(
              new Pose2d(0, 0, new Rotation2d(0)),
              List.of(),
              new Pose2d(2,0, new Rotation2d()),
              trajectoryConfig)
        );
    }
    public Command getTwoMeterBackwardTrajectory(){
        return generateTrajectory(
            TrajectoryGenerator.generateTrajectory(
              new Pose2d(0, 0, new Rotation2d(0)),
              List.of(),
              new Pose2d(2,0, new Rotation2d()),
              trajectoryConfig)
        );
    }

    public Command getModuleOneMeterPerSecond(){
        SwerveModuleState[] moduleStates = new SwerveModuleState[]{
            new SwerveModuleState(1,new Rotation2d(0)),
            new SwerveModuleState(1,new Rotation2d(0)),
            new SwerveModuleState(1,new Rotation2d(0)),
            new SwerveModuleState(1,new Rotation2d(0))
        };
        
        return new RunCommand(
            ()->rc.chassis.setModuleStates(moduleStates), 
            rc.chassis
        );
    }

    // public Command frSetDesiredState(double speedMetersPerSecond){
    //     return new RunCommand(
    //         ()->rc.chassis.frontRight.setDesiredState(new SwerveModuleState(speedMetersPerSecond, new Rotation2d())), 
    //         rc.chassis
    //     );
    // }

}
