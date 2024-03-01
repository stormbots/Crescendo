// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;
import java.util.ArrayList;
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

import edu.wpi.first.math.MathUtil;
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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
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

    ProfiledPIDController thetaController = new ProfiledPIDController(1/Math.PI/5*2, 0, 0, new TrapezoidProfile.Constraints(
        Math.PI, Math.PI));

    AutoBuilder autoBuilder;

    SendableChooser<Command> autoChooser = new SendableChooser<>();

    public PathPlannerPath pathPlannerPath = PathPlannerPath.fromPathFile("twoMeterAuto");

    Field2d pathPlannerField = new Field2d();

    public AutoFactory(RobotContainer rc){
        this.rc = rc;
        // initPathPlannerStuff();

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        autoChooser.setDefaultOption("Please Select Auto", new InstantCommand());
        autoChooser.addOption("blue center", 
            makeBasicAuto(
                new Pose2d(1.2, 5.55, new Rotation2d(Math.toRadians(0))), 
                new Pose2d(2.9, 5.55, new Rotation2d(Math.toRadians(0))))
            .andThen(rc.sequenceFactory.getTurnSetRPMandShootCommand(0, 8000, 25))
        );

        autoChooser.addOption("blue source", 
            makeBasicAuto(
                new Pose2d(0.8, 4.4, new Rotation2d(Math.toRadians(-60))),
                List.of(new Translation2d(1.0, 4.4-0.34)), 
                new Pose2d(2.9, 4.1, new Rotation2d(0)))
            .andThen(rc.sequenceFactory.getTurnSetRPMandShootCommand(-35, 8000, 20))
        );

        autoChooser.addOption("blue amp", 
            makeBasicAuto(
                new Pose2d(0.80, 6.60, new Rotation2d(Math.toRadians(60))), 
                List.of(new Translation2d(1.0, 6.6+0.34)), 
                new Pose2d(2.9, 7, new Rotation2d(Math.toRadians(0))))
            .andThen(rc.sequenceFactory.getTurnSetRPMandShootCommand(35, 8000, 20))
        );

        autoChooser.addOption("red center", 
            makeBasicAuto(
                new Pose2d(16.6-1.2, 5.55, new Rotation2d(Math.toRadians(180))), 
                new Pose2d(16.6-2.9, 5.55, new Rotation2d(Math.toRadians(180))))
            .andThen(rc.sequenceFactory.getTurnSetRPMandShootCommand(0, 8000, 20))
        );

        autoChooser.addOption("red source", 
            makeBasicAuto(
                new Pose2d(16.6-0.8, 4.4, new Rotation2d(Math.toRadians(-120))),
                List.of(new Translation2d(16.6-1.0, 4.4-0.34)), 
                new Pose2d(16.6-2.9, 4.1, new Rotation2d(Math.toRadians(180))))
            .andThen(rc.sequenceFactory.getTurnSetRPMandShootCommand(35, 8000, 18))
        );

        autoChooser.addOption("red amp", 
            makeBasicAuto(
                new Pose2d(16.6-0.80, 6.60, new Rotation2d(Math.toRadians(120))), 
                List.of(new Translation2d(16.6-1.0, 6.6+0.34)), 
                new Pose2d(16.6-2.9, 7, new Rotation2d(Math.toRadians(180))))
            .andThen(rc.sequenceFactory.getTurnSetRPMandShootCommand(-35, 8000, 18))
        );

        autoChooser.addOption("blue amp drive to mid", 
            makeBasicAuto(
                new Pose2d(0.80, 6.60, new Rotation2d(Math.toRadians(60))), 
                List.of(new Translation2d(1.0, 6.6+0.34)), 
                new Pose2d(2.9, 7, new Rotation2d(Math.toRadians(0))))
            .andThen(rc.sequenceFactory.getTurnSetRPMandShootCommand(35, 8000, 20))
            .andThen(
                makeNoteToMidIntake(
                    new Pose2d(2.9, 7, new Rotation2d()), 
                    new Pose2d(8.3, 7.45, new Rotation2d()))
            )
        );

        autoChooser.addOption("vv Test Autos vv", new InstantCommand());

        autoChooser.addOption("rpmandShoot", rc.sequenceFactory.getSetRPMandShootCommand(3000, 20));


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

    SwerveControllerCommand generateSwerveControllerCommand(Pose2d start, List<Translation2d> midpoints, Pose2d end){
        return generateSwerveControllerCommand(
            TrajectoryGenerator.generateTrajectory(start, midpoints, end, trajectoryConfig)
        );
    }

    public Command makeBasicAuto(Pose2d start, List<Translation2d> midpoints,Pose2d end){
        //Setting gyro offset + odometry
        //Preparing shooting
        //Shooting
        //Running path to note + intake

        //todo Shooting for the second time
        double gyroOffset = 0;
        if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue){//Checking if blue
            gyroOffset = -start.getRotation().getRadians();
        }
        else{
            gyroOffset = MathUtil.angleModulus(Math.PI-start.getRotation().getRadians());
        }
        final double fgyroOffset = Math.toDegrees(gyroOffset);

        return new InstantCommand()
        .andThen(()->rc.chassis.setFieldCentricOffset(fgyroOffset))
        .andThen(()->rc.chassis.resetOdometry(start))
        // .andThen(()->rc.chassis.resetOdometry(new Pose2d(start.getX(),start.getY(), rc.navx.getRotation2d())))
        .andThen(rc.sequenceFactory.getSetRPMandShootCommand(5500, 50))
        .andThen(new ParallelDeadlineGroup(
            generateSwerveControllerCommand(start, midpoints, end).andThen(new WaitCommand(1)).withTimeout(6),
            rc.sequenceFactory.getIntakeThenAlignCommand()
        ))
        ;

    }

    public Command makeBasicAuto(Pose2d start, Pose2d end){
        return makeBasicAuto(start, List.of(), end);
    }



    public SendableChooser<Command> getAutoChooser(){
        return autoChooser;
    }

    // Example
    // public Command getBlueBottomAuto(){
    //     return new InstantCommand()
    //     .andThen(rc.sequenceFactory.getSetRPMandShootCommand(6000, 45))
    //     .andThen(new ParallelDeadlineGroup(
    //         blueBotStartToBotNotePathCommand().withTimeout(5).andThen(new WaitCommand(1)), 
    //         rc.sequenceFactory.getIntakeThenAlignCommand()
    //     ))
    //     .andThen(rc.sequenceFactory.getTurnSetRPMandShootCommand(Math.toRadians(-25), 8000, 20))
    //     ;

    // }

    // // obsolete (for now)
    // public Command getTwoMeterPathPlanner(){
    //     //reset positon
    //     var path = AutoBuilder.followPath(pathPlannerPath);
    //     var start = pathPlannerPath.getPreviewStartingHolonomicPose();
        
    //     return new InstantCommand()
    //     .andThen( ()-> rc.chassis.setFieldCentricOffset(-90) )
    //     .andThen( ()->rc.chassis.resetOdometry(new Pose2d(start.getX(),start.getY(),rc.navx.getRotation2d())) )
    //     .andThen(path);
        
    //     // return AutoBuilder.followPath(pathPlannerPath);
    // }


}
