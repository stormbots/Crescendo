// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ChassisConstants.AutoConstants;
import frc.robot.ChassisConstants.DriveConstants;




/** Add your docs here. */
public class AutoFactory {
    RobotContainer rc;


    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        Units.MetersPerSecond.of(AutoConstants.kMaxSpeedMetersPerSecond), 
        Units.MetersPerSecondPerSecond.of(AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    );

    ProfiledPIDController thetaController = new ProfiledPIDController(1/Math.PI/10, 0, 0, new TrapezoidProfile.Constraints(
        Math.PI, Math.PI));

    AutoBuilder autoBuilder;

    SendableChooser<Command> autoChooser = new SendableChooser<>();

    public PathPlannerPath pathPlannerPath = PathPlannerPath.fromPathFile("twoMeterAuto");

    Field2d pathPlannerField = new Field2d();

    public AutoFactory(RobotContainer rc){
        this.rc = rc;
        // initTrajectoryStuff();
        initPathPlannerStuff();

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        autoChooser.setDefaultOption("Please Select Auto", new InstantCommand());
        autoChooser.addOption("blue bottom", getBottomAuto());
        SmartDashboard.putData("Auto Chooser", autoChooser);

    }

    void initPathPlannerStuff(){
        Supplier<ChassisSpeeds> getSpeed = ()->{
            return rc.swerveDriveKinematics.toChassisSpeeds(rc.chassis.getModuleStates());
        };

        Consumer<ChassisSpeeds> setSpeed = (chassisSpeeds)->{
            var swerveModuleStates = rc.swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            rc.chassis.setModuleStates(swerveModuleStates);
        };

        HolonomicPathFollowerConfig holonomicPathFollowerConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(2/1*2), 
            new PIDConstants(1/Math.PI*2), 
            DriveConstants.kMaxSpeedMetersPerSecond, 
            DriveConstants.distanceToModuleFromCenter,
            new ReplanningConfig(true, false));

        BooleanSupplier shouldFlipPath = () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followe
            // d to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
        };

        AutoBuilder.configureHolonomic(
            rc.chassis::getPose,
            rc.chassis::resetOdometry,
            getSpeed, 
            setSpeed, 
            holonomicPathFollowerConfig, 
            shouldFlipPath, 
            rc.chassis
        );

        SmartDashboard.putData("Field", pathPlannerField);

        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            pathPlannerField.setRobotPose(pose);
        });

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            pathPlannerField.getObject("target pose").setPose(pose);
        });

        PathPlannerLogging.setLogActivePathCallback((list)->pathPlannerField.getObject("path").setPoses(list));

    }

    public Command ExampleAuto(){
        return new InstantCommand(()->{},rc.chassis,rc.climber);
    }


    //trajectory lib: Currently not used
    SwerveControllerCommand generateSwerveControllerCommand(Trajectory trajectory){
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
          new PIDController(AutoConstants.kPDrivingP, 0, 0),
          new PIDController(AutoConstants.kPDrivingP, 0, 0),
          thetaController,
          rc.chassis::setModuleStates,
          rc.chassis
        );
    
        return swerveControllerCommand;
    }

    public Command getTwoMeterForwardTrajectory(){
        return 
        new InstantCommand()
            .andThen(()->rc.chassis.setFieldCentricOffset(-90))
            .andThen(()->rc.chassis.resetOdometry(new Pose2d(0,0, rc.navx.getRotation2d())))
            .andThen(
                generateSwerveControllerCommand(
                    TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0, 0, new Rotation2d(Math.PI/2)),
                    List.of(),
                    new Pose2d(2,0, new Rotation2d()),
                    trajectoryConfig)
                )
            )
            .andThen(
                generateSwerveControllerCommand(
                    TrajectoryGenerator.generateTrajectory(
                    new Pose2d(2, 0, new Rotation2d(Math.PI/2)),
                    List.of(),
                    new Pose2d(2,2, new Rotation2d()),
                    trajectoryConfig)
                )
            );
    }
    // public Command getTwoMeterBackwardTrajectory(){
    //     return generateSwerveControllerCommand(
    //         TrajectoryGenerator.generateTrajectory(
    //           rc.chassis.getPose(),
    //           List.of(),
    //           new Pose2d(0,0, new Rotation2d()),
    //           trajectoryConfig)
    //     );
    // }

    // public Command getModuleOneMeterPerSecond(){
    //     SwerveModuleState[] moduleStates = new SwerveModuleState[]{
    //         new SwerveModuleState(1,new Rotation2d(0)),
    //         new SwerveModuleState(1,new Rotation2d(0)),
    //         new SwerveModuleState(1,new Rotation2d(0)),
    //         new SwerveModuleState(1,new Rotation2d(0))
    //     };
        
    //     return new RunCommand(
    //         ()->rc.chassis.setModuleStates(moduleStates), 
    //         rc.chassis
    //     );
    // }

    // public Command frSetDesiredState(double speedMetersPerSecond){
    //     return new RunCommand(
    //         ()->rc.chassis.frontRight.setDesiredState(new SwerveModuleState(speedMetersPerSecond, new Rotation2d())), 
    //         rc.chassis
    //     );
    // }

    public SendableChooser<Command> getAutoChooser(){
        return autoChooser;
    }


    // public Command getAutoCommand(){
    //     //figure out what we want to do from the positon
    // }

    public Command getBottomAuto(){
        return new InstantCommand()
        .andThen(rc.sequenceFactory.getSetRPMandShootCommand(6000, 50))
        .andThen(new ParallelDeadlineGroup(
            botStartToBotNotePathCommand().withTimeout(5).andThen(new WaitCommand(1)), 
            rc.sequenceFactory.getIntakeThenAlignCommand()
        ));

            //Ideally we would want to shoot this note, but shooter stuff isnt even up yet so what am i supposed to do
    }

    public Command getTwoMeterPathPlanner(){
        //reset positon
        var path = AutoBuilder.followPath(pathPlannerPath);
        var start = pathPlannerPath.getPreviewStartingHolonomicPose();
        
        return new InstantCommand()
        .andThen( ()-> rc.chassis.setFieldCentricOffset(-90) )
        .andThen( ()->rc.chassis.resetOdometry(new Pose2d(start.getX(),start.getY(),rc.navx.getRotation2d())) )
        .andThen(path);
        
        // return AutoBuilder.followPath(pathPlannerPath);
    }

    public Command botStartToBotNotePathCommand(){
        return new InstantCommand()
            .andThen(()->rc.chassis.setFieldCentricOffset(60))
            .andThen(()->rc.chassis.resetOdometry(new Pose2d(0.75,4.55, rc.navx.getRotation2d())))
            .andThen(
                generateSwerveControllerCommand(
                    TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0.75, 4.55, new Rotation2d(Math.toRadians(-60))),
                    List.of(),
                    new Pose2d(2.70,4.10, new Rotation2d()),
                    trajectoryConfig)
                )
            );
    }

    public Command botNoteToBotSharePathCommand(){
        return new InstantCommand()
            .andThen(
                generateSwerveControllerCommand(
                    TrajectoryGenerator.generateTrajectory(
                    new Pose2d(2.70, 4.10, new Rotation2d(Math.toRadians(-40))),
                    List.of(new Translation2d(2.70,1.90)),
                    new Pose2d(8.30, 0.80, new Rotation2d()),
                    trajectoryConfig)
                )
            );
    }
    

}
