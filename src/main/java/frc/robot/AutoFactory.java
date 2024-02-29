// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
        // initTrajectoryStuff();
        initPathPlannerStuff();

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        autoChooser.setDefaultOption("Please Select Auto", new InstantCommand());
        autoChooser.addOption("blue center", makeBasicAuto(new Pose2d(1.2, 5.55, new Rotation2d(0)), new Pose2d(2.9, 5.55, new Rotation2d(0))));
        autoChooser.addOption("blue source", makeBasicAuto(new Pose2d(0.8, 4.4, new Rotation2d(-60)),List.of(new Translation2d(0.8, 4.2)), new Pose2d(2.9, 4.1, new Rotation2d(0))));
        autoChooser.addOption("blue amp", makeBasicAuto(new Pose2d(0.80, 6.60, new Rotation2d(60)), List.of(new Translation2d(0.9, 6.8)), new Pose2d(2.9, 7, new Rotation2d(0))));
        autoChooser.addOption("shootandrpm", rc.sequenceFactory.getSetRPMandShootCommand(6000, 45));
        autoChooser.addOption("turnandshootandrpm", rc.sequenceFactory.getTurnSetRPMandShootCommand(-20, 6000, 45));


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
        final double fgyroOffset = gyroOffset;

        return new InstantCommand()
        .andThen(()->rc.chassis.setFieldCentricOffset(fgyroOffset))
        .andThen(()->rc.chassis.resetOdometry(new Pose2d(start.getX(),start.getY(), rc.navx.getRotation2d())))
        .andThen(rc.sequenceFactory.getSetRPMandShootCommand(6000, 45))
        .andThen(new ParallelDeadlineGroup(
            generateSwerveControllerCommand(start, midpoints, end).withTimeout(5),
            rc.sequenceFactory.getIntakeThenAlignCommand()
        ))
        ;

    }

    public Command makeBasicAuto(Pose2d start, Pose2d end){
        return makeBasicAuto(start, List.of(), end);
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

    public SendableChooser<Command> getAutoChooser(){
        return autoChooser;
    }


    public Command getBlueBottomAuto(){
        return new InstantCommand()
        .andThen(rc.sequenceFactory.getSetRPMandShootCommand(6000, 45))
        .andThen(new ParallelDeadlineGroup(
            blueBotStartToBotNotePathCommand().withTimeout(5).andThen(new WaitCommand(1)), 
            rc.sequenceFactory.getIntakeThenAlignCommand()
        ))
        .andThen(rc.sequenceFactory.getTurnSetRPMandShootCommand(Math.toRadians(-25), 8000, 20))
        ;

    }

    public Command getBlueMidAuto(){
        return new InstantCommand()
        .andThen(rc.sequenceFactory.getSetRPMandShootCommand(6000, 45))
        .andThen(new ParallelDeadlineGroup(
            blueMidStartToMidNotePathCommand().withTimeout(5).andThen(new WaitCommand(1)), 
            rc.sequenceFactory.getIntakeThenAlignCommand()
        ))
        .andThen(rc.sequenceFactory.getTurnSetRPMandShootCommand(Math.toRadians(0), 8000, 20))
        ;

    }

    public Command getBlueTopAuto(){
        return new InstantCommand()
        .andThen(rc.sequenceFactory.getSetRPMandShootCommand(6000, 45))
        .andThen(new ParallelDeadlineGroup(
            blueTopStartToTopNotePathCommand().withTimeout(5).andThen(new WaitCommand(1)), 
            rc.sequenceFactory.getIntakeThenAlignCommand()
        ))
        .andThen(rc.sequenceFactory.getTurnSetRPMandShootCommand(Math.toRadians(27), 8000, 20))
        ;

    }

    public Command getRedBottomAuto(){
        return new InstantCommand()
        .andThen(rc.sequenceFactory.getSetRPMandShootCommand(6000, 45))
        .andThen(new ParallelDeadlineGroup(
            blueBotStartToBotNotePathCommand().withTimeout(5).andThen(new WaitCommand(1)), 
            rc.sequenceFactory.getIntakeThenAlignCommand()
        ))
        .andThen(rc.sequenceFactory.getTurnSetRPMandShootCommand(Math.toRadians(25), 8000, 20))
        ;

    }

    public Command getRedMidAuto(){
        return new InstantCommand()
        .andThen(rc.sequenceFactory.getSetRPMandShootCommand(6000, 45))
        .andThen(new ParallelDeadlineGroup(
            blueMidStartToMidNotePathCommand().withTimeout(5).andThen(new WaitCommand(1)), 
            rc.sequenceFactory.getIntakeThenAlignCommand()
        ))
        .andThen(rc.sequenceFactory.getTurnSetRPMandShootCommand(Math.toRadians(0), 8000, 20))
        ;

    }

    public Command getRedTopAuto(){
        return new InstantCommand()
        .andThen(rc.sequenceFactory.getSetRPMandShootCommand(6000, 45))
        .andThen(new ParallelDeadlineGroup(
            blueTopStartToTopNotePathCommand().withTimeout(5).andThen(new WaitCommand(1)), 
            rc.sequenceFactory.getIntakeThenAlignCommand()
        ))
        .andThen(rc.sequenceFactory.getTurnSetRPMandShootCommand(Math.toRadians(-27), 8000, 20))
        ;

    }

    /*
     * Generates a auto sequence managing start position, end position, shooting, and trajectory . 
     * Boolean isBlue is uneccesary but added for the sake of preventing unwanted behaviors
     */
    public Command makeStartToNoteAutoSequence(Pose2d startPose, Pose2d endPose, boolean isBlue){
        var path =  TrajectoryGenerator.generateTrajectory(
            startPose,
            List.of(),
            endPose,
            trajectoryConfig);

        var pathCommand = generateSwerveControllerCommand(path);

        double offset = 0;

        if(isBlue){
            offset = -startPose.getRotation().getRadians();
        }
        else{
            offset = Math.PI - startPose.getRotation().getRadians();
        }

        offset = Math.toDegrees(MathUtil.angleModulus(offset));
        final double foffset = offset;

        return new InstantCommand()
        .andThen(()->rc.chassis.setFieldCentricOffset(foffset))

        .andThen(rc.sequenceFactory.getSetRPMandShootCommand(6000, 45))
        .andThen(new ParallelDeadlineGroup(
            pathCommand.withTimeout(path.getTotalTimeSeconds()+1).andThen(new WaitCommand(1)), 
            rc.sequenceFactory.getIntakeThenAlignCommand()
        ));
    }

    /*
     * Generates a auto sequence managing start position, end position, shooting, and trajectory . 
     * Boolean isBlue is uneccesary but added for the sake of preventing unwanted behaviors
     * Overloaded constructor
     */

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

    public Command blueBotStartToBotNotePathCommand(){
        return new InstantCommand()
            .andThen(()->rc.chassis.setFieldCentricOffset(60))
            .andThen(()->rc.chassis.resetOdometry(new Pose2d(0.75,4.55, rc.navx.getRotation2d())))
            .andThen(
                generateSwerveControllerCommand(
                    TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0.75, 4.55, new Rotation2d(Math.toRadians(-60))),
                    List.of(),
                    new Pose2d(2.80,4.25, new Rotation2d()),
                    trajectoryConfig)
                )
            );
    }

    public Command blueBotNoteToBotSharePathCommand(){
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

    public Command blueMidStartToMidNotePathCommand(){
        return new InstantCommand()
            .andThen(()->rc.chassis.setFieldCentricOffset(0))
            .andThen(()->rc.chassis.resetOdometry(new Pose2d(1.2,5.7, rc.navx.getRotation2d())))
            .andThen(
                generateSwerveControllerCommand(
                    TrajectoryGenerator.generateTrajectory(
                    new Pose2d(1.2,5.7, new Rotation2d(Math.toRadians(0))),
                    List.of(),
                    new Pose2d(2.90,5.55, new Rotation2d()),
                    trajectoryConfig)
                )
            );
    }

    public Command blueTopStartToTopNotePathCommand(){
        return new InstantCommand()
            .andThen(()->rc.chassis.setFieldCentricOffset(-60))
            .andThen(()->rc.chassis.resetOdometry(new Pose2d(0.75,6.55, rc.navx.getRotation2d())))
            .andThen(
                generateSwerveControllerCommand(
                    TrajectoryGenerator.generateTrajectory(
                    new Pose2d(0.75, 6.55, new Rotation2d(Math.toRadians(60))),
                    List.of(),
                    new Pose2d(2.90,7.00, new Rotation2d()),
                    trajectoryConfig)
                )
            );
    }

    public Command redBotStartToBotNotePathCommand(){
        return new InstantCommand()
            .andThen(()->rc.chassis.setFieldCentricOffset(120))
            .andThen(()->rc.chassis.resetOdometry(new Pose2d(16-0.75,4.55, rc.navx.getRotation2d())))
            .andThen(
                generateSwerveControllerCommand(
                    TrajectoryGenerator.generateTrajectory(
                    new Pose2d(16-0.75, 4.55, new Rotation2d(Math.toRadians(-120))),
                    List.of(),
                    new Pose2d(16-2.80,4.25, new Rotation2d()),
                    trajectoryConfig)
                )
            );
    }

    public Command redMidStartToMidNotePathCommand(){
        return new InstantCommand()
            .andThen(()->rc.chassis.setFieldCentricOffset(-180))
            .andThen(()->rc.chassis.resetOdometry(new Pose2d(16-1.2,5.7, rc.navx.getRotation2d())))
            .andThen(
                generateSwerveControllerCommand(
                    TrajectoryGenerator.generateTrajectory(
                    new Pose2d(16-1.2,5.7, new Rotation2d(Math.toRadians(180))),
                    List.of(),
                    new Pose2d(16-2.90,5.55, new Rotation2d()),
                    trajectoryConfig)
                )
            );
    }

    public Command redTopStartToTopNotePathCommand(){
        return new InstantCommand()
            .andThen(()->rc.chassis.setFieldCentricOffset(-120))
            .andThen(()->rc.chassis.resetOdometry(new Pose2d(16-0.75,6.55, rc.navx.getRotation2d())))
            .andThen(
                generateSwerveControllerCommand(
                    TrajectoryGenerator.generateTrajectory(
                    new Pose2d(16-0.75, 6.55, new Rotation2d(Math.toRadians(120))),
                    List.of(),
                    new Pose2d(16-2.90,7.00, new Rotation2d()),
                    trajectoryConfig)
                )
            );
    }


}
