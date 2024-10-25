// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Future;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

import javax.print.attribute.standard.RequestingUserName;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import com.stormbots.LUT;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.ChassisConstants.AutoConstants;
import frc.robot.ChassisConstants.DriveConstants;
import frc.robot.commands.PassthroughAlignNote;
import frc.robot.commands.SetDunkArmSlew;
import frc.robot.commands.ShooterSetVision;
import frc.robot.commands.VisionTrackNoteAuto;
import frc.robot.commands.VisionTurnToAprilTag;
import frc.robot.commands.VisionTurnToSpeakerOpticalOnly;
import frc.robot.subsystems.PassthroughLock;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterVision;
import frc.robot.subsystems.IntakeVision.IntakePipeline;




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

    Future<Boolean> initAutoChooserFuture = CompletableFuture.supplyAsync(()->true);

    final double intakeVisionAutoTimeout = 1.5;
    final double shooterVisionAutoTimeout = 2.0;


    public AutoFactory(RobotContainer rc){
        this.rc = rc;
        initPathPlannerStuff();

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        
        // initAutoChooser(); //aktes too long, run in background
        rebuild();

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public void rebuild(){
        initAutoChooserFuture = CompletableFuture.supplyAsync(this::initAutoChooser);
    }

    boolean initAutoChooser(){
        BooleanSupplier isBlue = () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Blue;
            }
            return false;
        };

        // BooleanSupplier isOverNoteSeeingLimit = ()->{
        //     double poseX = rc.swerveDrivePoseEstimator.getEstimatedPosition().getX();
        //     return Math.abs(rc.swerveDrivePoseEstimator.getEstimatedPosition().getX()-(16.542/2)) < ((16.542/2)-5.474);
        // };

        autoChooser.setDefaultOption("Please Select Auto", new InstantCommand());

        // autoChooser.addOption("vv Test Autos vv", new InstantCommand());

        // autoChooser.addOption("5NoteAmpChoreo", new InstantCommand()
        //     .andThen(()->rc.chassis.setFieldCentricOffset(-60, isBlue))
        //     .andThen(new PathPlannerAuto("ChoreoAmpAuto"))
        // );

        // autoChooser.addOption("4NoteAmpRushChoreo", new InstantCommand()
        //     .andThen(()->rc.chassis.setFieldCentricOffset(0, isBlue))
        //     .andThen(new PathPlannerAuto("FarStartChoreoAmpRushAuto"))
        // );

        // autoChooser.addOption("4NoteSourceRushChoreo", new InstantCommand()
        //     .andThen(()->rc.chassis.setFieldCentricOffset(0, isBlue))
        //     .andThen(new PathPlannerAuto("ChoreoSourceRushAuto"))
        // );
        // autoChooser.addOption("FarStart4NoteSourceRushChoreo", new InstantCommand()
        //     .andThen(()->rc.chassis.setFieldCentricOffset(0, isBlue))
        //     .andThen(new PathPlannerAuto("FarStartChoreoSourceRushAuto"))
        // );

        // autoChooser.addOption("5NoteCenterTopFirstChoreo", new InstantCommand()
        //     .andThen(()->rc.chassis.setFieldCentricOffset(0, isBlue))
        //     .andThen(new PathPlannerAuto("ChoreoCenterTopFirstAuto"))
        // );

        autoChooser.addOption("SourceVisionAuto", 
            new SequentialCommandGroup(
                new InstantCommand(()->rc.chassis.setFieldCentricOffset(0, isBlue)),
                new InstantCommand(()->rc.chassis.resetOdometryAllianceManaged(new Pose2d(1.5,3.589,new Rotation2d()))),
                makeVisionAuto(PathPlannerPath.fromChoreoTrajectory("FarStartSourceRunThrough"), PathPlannerPath.fromChoreoTrajectory("SourceReturnPath"), 1.3, 14, 5500),
                makeVisionAuto(PathPlannerPath.fromChoreoTrajectory("SourceRunThrough"), PathPlannerPath.fromChoreoTrajectory("SourceReturnPath")),
                makeVisionAuto(PathPlannerPath.fromChoreoTrajectory("SourceRunThrough"), PathPlannerPath.fromChoreoTrajectory("SourceReturnPath"))
            )
        );

        // autoChooser.addOption("SourceVisionAutoReverse", 
        //     new SequentialCommandGroup(
        //         new InstantCommand(()->rc.chassis.setFieldCentricOffset(0, isBlue)),
        //         new InstantCommand(()->rc.chassis.resetOdometryAllianceManaged(new Pose2d(1.5,3.589,new Rotation2d()))),
        //         makeVisionAuto(PathPlannerPath.fromChoreoTrajectory("FarStartSourceRunThroughReverse"), PathPlannerPath.fromChoreoTrajectory("SourceReturnPath"), 1.1, 14, 5500),
        //         makeVisionAuto(PathPlannerPath.fromChoreoTrajectory("SourceRunThroughReverse"), PathPlannerPath.fromChoreoTrajectory("SourceReturnPath")),
        //         makeVisionAuto(PathPlannerPath.fromChoreoTrajectory("SourceRunThrough"), PathPlannerPath.fromChoreoTrajectory("SourceReturnPath"))
        //     )
        // );

        // autoChooser.addOption("FarSourceVisionAuto", 
        //     new SequentialCommandGroup(
        //         new InstantCommand(()->rc.chassis.setFieldCentricOffset(0, isBlue)),
        //         new InstantCommand(()->rc.chassis.resetOdometryAllianceManaged(new Pose2d(1.5,3.589,new Rotation2d()))),
        //         makeVisionAuto(PathPlannerPath.fromChoreoTrajectory("SuperFarStartSourceRunThrough"), PathPlannerPath.fromChoreoTrajectory("SourceReturnPath")),
        //         makeVisionAuto(PathPlannerPath.fromChoreoTrajectory("SourceRunThrough"), PathPlannerPath.fromChoreoTrajectory("SourceReturnPath")),
        //         makeVisionAuto(PathPlannerPath.fromChoreoTrajectory("SourceRunThrough"), PathPlannerPath.fromChoreoTrajectory("SourceReturnPath"))
        //     )
        // );

        // autoChooser.addOption("AmpVisionAuto", 
        //     new SequentialCommandGroup(
        //         new InstantCommand(()->rc.chassis.setFieldCentricOffset(0, isBlue)),
        //         new InstantCommand(()->rc.chassis.resetOdometryAllianceManaged(new Pose2d(1.488,6.52,new Rotation2d()))),
        //         makeVisionAuto(PathPlannerPath.fromChoreoTrajectory("FarStartAmpRunThrough"), PathPlannerPath.fromChoreoTrajectory("AmpReturnPath"), 1.2, 14, 5500),
        //         makeVisionAuto(PathPlannerPath.fromChoreoTrajectory("AmpRunThrough"), PathPlannerPath.fromChoreoTrajectory("AmpReturnPath")),
        //         makeVisionAuto(PathPlannerPath.fromChoreoTrajectory("AmpRunThrough"), PathPlannerPath.fromChoreoTrajectory("AmpReturnPath"))
        //     )
        // );

        autoChooser.addOption("MidSafeVisionAuto", 
            new SequentialCommandGroup(
                new InstantCommand(()->rc.chassis.setFieldCentricOffset(0, isBlue)),
                new InstantCommand(()->rc.chassis.resetOdometryAllianceManaged(new Pose2d(1.35,5.48,new Rotation2d()))),
                new ShooterSetVision(rc.shooter, rc.shooterVision, rc.shooterFlywheel).withTimeout(3),
                new ParallelCommandGroup(
                    new RunCommand(rc.intake::intake, rc.intake).finallyDo(rc.intake::stop), 
                    new RunCommand(rc.passthrough::intake, rc.passthrough).finallyDo(rc.passthrough::stop)
                )
                .until(()->!rc.passthrough.isBlocked()&&!rc.intake.isBlocked()),
                makeVisionAuto(PathPlannerPath.fromChoreoTrajectory("CenterToMidNote"), new Pose2d(1.35,5.48,new Rotation2d())),
                makeVisionAuto(PathPlannerPath.fromChoreoTrajectory("CenterToTopNote"), new Pose2d(1.35,5.48,new Rotation2d())),
                makeVisionAuto(PathPlannerPath.fromChoreoTrajectory("CenterToBotNote"), new Pose2d(1.35,5.48,new Rotation2d())),
                makeVisionAuto(PathPlannerPath.fromChoreoTrajectory("CenterCheckRemaining"), new Pose2d(1.35,5.48,new Rotation2d()))

            )
        );

        // autoChooser.addOption("vv SysID vv", new InstantCommand());
        
        // autoChooser.addOption("quasi forward", rc.chassis.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption("quasi backward", rc.chassis.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // autoChooser.addOption("dynamic forward", rc.chassis.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption("dynamic backward", rc.chassis.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // autoChooser.addOption("sysid Flywheel routine", 
        //     new InstantCommand()
        //         .andThen(rc.shooterFlywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward).withTimeout(5))
        //         .andThen(new WaitCommand(5))
        //         .andThen(rc.shooterFlywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).withTimeout(5))
        //         .andThen(new WaitCommand(5))
        //         .andThen(rc.shooterFlywheel.sysIdDynamic(SysIdRoutine.Direction.kForward).withTimeout(2))
        //         .andThen(new WaitCommand(5))
        //         .andThen(rc.shooterFlywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse).withTimeout(2))
        //         .andThen(new WaitCommand(5))

        // );

        // autoChooser.addOption("sysid shooter routine", 
        //     new InstantCommand()
        //         .andThen(rc.shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward).withTimeout(2))
        //         .andThen(new WaitCommand(2))
        //         .andThen(rc.shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).withTimeout(2))
        //         .andThen(new WaitCommand(2))
        //         .andThen(rc.shooter.sysIdDynamic(SysIdRoutine.Direction.kForward).withTimeout(1))
        //         .andThen(new WaitCommand(2))
        //         .andThen(rc.shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse).withTimeout(1))
        //         .andThen(new WaitCommand(2))

        // );

        // autoChooser.addOption("sysid dunkArm routine", 
        //     new InstantCommand()
        //         .andThen(rc.dunkArm.sysIdQuasistatic(SysIdRoutine.Direction.kForward).withTimeout(2))
        //         .andThen(new WaitCommand(2))
        //         .andThen(rc.dunkArm.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).withTimeout(2))
        //         .andThen(new WaitCommand(2))
        //         .andThen(rc.dunkArm.sysIdDynamic(SysIdRoutine.Direction.kForward).withTimeout(2))
        //         .andThen(new WaitCommand(2))
        //         .andThen(rc.dunkArm.sysIdDynamic(SysIdRoutine.Direction.kReverse).withTimeout(2))
        //         .andThen(new WaitCommand(2))

        // );

        // autoChooser.addOption("testNoteVisionChoreo", 
        //     new InstantCommand(()->rc.chassis.setFieldCentricOffset(0, isBlue))
        //     .andThen(new InstantCommand(()->rc.chassis.resetOdometryAllianceManaged(new Pose2d(1.5,3.589,new Rotation2d()))))
        //     .andThen(new InstantCommand(()->rc.shooterVision.enableAutoVision(true)))
            
        // );
        

        return false; //will bryan cry tonight YES!!!!!!! I WILL BE!!!!!!
        
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
            new PIDConstants(2.5), 
            new PIDConstants(5*0.5), 
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

        LUT shooterLUT = Shooter.normalLUT;
        Measure<Distance> distance = Units.Meters.of(Math.hypot(4.3-0.4, 6.25-5.55));
        
        var angle = shooterLUT.get(distance.in(Units.Inches))[0];
        var rpm = shooterLUT.get(distance.in(Units.Inches))[1];

        NamedCommands.registerCommand("intakeAndAlign", rc.sequenceFactory.getIntakeThenAlignCommand().finallyDo((e)->PassthroughLock.getInstance().unlock()));
        NamedCommands.registerCommand("intakeFull", new ParallelCommandGroup(new RunCommand(rc.intake::intake, rc.intake), new RunCommand(rc.passthrough::intake, rc.passthrough)));
        NamedCommands.registerCommand("intakeShoot", 
            new ParallelCommandGroup(
                new RunCommand(rc.intake::intake, rc.intake), 
                new RunCommand(rc.passthrough::intake, rc.passthrough)
            ).until(()->!rc.passthrough.isBlocked()&&!rc.intake.isBlocked()));
        NamedCommands.registerCommand("intakeStop", rc.sequenceFactory.getStopIntakingCommand().withTimeout(5));

        NamedCommands.registerCommand("subwooferShot", rc.sequenceFactory.getSetRPMandShootCommand(5500, 45));
        NamedCommands.registerCommand("spinUpShotNoStop", rc.sequenceFactory.getToShooterStateCommand(7000, 32.5));
        NamedCommands.registerCommand("stageShot", rc.sequenceFactory.getSetRPMandShootCommand(8000, 20));
        NamedCommands.registerCommand("stageShotNoStop", rc.sequenceFactory.getToShooterStateCommand(8000, 20));
        NamedCommands.registerCommand("stopFlywheel", new WaitCommand(0.1).andThen(new InstantCommand(()->rc.shooterFlywheel.setRPM(0))));
        NamedCommands.registerCommand("dunkArmUp", new SetDunkArmSlew(80, rc.dunkArm));
        NamedCommands.registerCommand("dunkArmDown", new SetDunkArmSlew(-25, rc.dunkArm));

        NamedCommands.registerCommand("servoDown", PassthroughLock.setUnlocked());
        NamedCommands.registerCommand("servoUp", PassthroughLock.setLocked());


        // NamedCommands.registerCommand("topSpinUpShotNoStop", new ShooterSetOdometry(rc.shooter, rc.shooterFlywheel, new Pose2d(2.15, 6.5, new Rotation2d())).runForever());
        // NamedCommands.registerCommand("midSpinUpShotNoStop", new ShooterSetOdometry(rc.shooter, rc.shooterFlywheel, new Pose2d(2.15, 6.5, new Rotation2d())).runForever());
        // NamedCommands.registerCommand("botSpinUpShotNoStop", new ShooterSetOdometry(rc.shooter, rc.shooterFlywheel, new Pose2d(2.15, 6.5, new Rotation2d())).runForever());

        // NamedCommands.registerCommand("topNoteShotNoStop", new ShooterSetOdometry(rc.shooter, rc.shooterFlywheel, new Pose2d(2.9, 7, new Rotation2d())).runForever());
        // NamedCommands.registerCommand("midNoteShotNoStop", new ShooterSetOdometry(rc.shooter, rc.shooterFlywheel, new Pose2d(2.9, 5.55, new Rotation2d())).runForever());
        // NamedCommands.registerCommand("botNoteShotNoStop", new ShooterSetOdometry(rc.shooter, rc.shooterFlywheel, new Pose2d(2.78, 4.9, new Rotation2d())).runForever());

        // NamedCommands.registerCommand("topShootPosShotNoStop", new ShooterSetOdometry(rc.shooter, rc.shooterFlywheel, new Pose2d(4.3, 6.25, new Rotation2d())).runForever());
        // NamedCommands.registerCommand("midShootPosShotNoStop", new ShooterSetOdometry(rc.shooter, rc.shooterFlywheel, new Pose2d(4.3, 4.85, new Rotation2d())).runForever());
        // NamedCommands.registerCommand("botShootPosShotNoStop", new ShooterSetOdometry(rc.shooter, rc.shooterFlywheel, new Pose2d(3, 2.8, new Rotation2d())).runForever());

        NamedCommands.registerCommand("topSpinUpShotNoStopOnTheRun", rc.sequenceFactory.getToShooterStateCommand(5000, 28));
        NamedCommands.registerCommand("topNoteShotNoStopOnTheRun", rc.sequenceFactory.getToShooterStateCommand(5000, 18));

        NamedCommands.registerCommand("topSpinUpShotNoStop", rc.sequenceFactory.getToShooterStateCommand(4000, 28.5-0.5));
        NamedCommands.registerCommand("midSpinUpShotNoStop", rc.sequenceFactory.getToShooterStateCommand(4000, 32));
        NamedCommands.registerCommand("botSpinUpShotNoStop", rc.sequenceFactory.getToShooterStateCommand(4000, 26)); //previously 28 but we only use sourceRush auto for source side

        NamedCommands.registerCommand("topNoteShotNoStop", rc.sequenceFactory.getToShooterStateCommand(5000, 20.7));
        NamedCommands.registerCommand("midNoteShotNoStop", rc.sequenceFactory.getToShooterStateCommand(5000, 23));
        NamedCommands.registerCommand("botNoteShotNoStop", rc.sequenceFactory.getToShooterStateCommand(5000, 20.5));

        NamedCommands.registerCommand("topShootPosShotNoStop", rc.sequenceFactory.getToShooterStateCommand(5500, 14));
        NamedCommands.registerCommand("topShootPosShotNoStop3", rc.sequenceFactory.getToShooterStateCommand(5500, 14));
        NamedCommands.registerCommand("topShootPosShotNoStop4", rc.sequenceFactory.getToShooterStateCommand(5500, 14));
        NamedCommands.registerCommand("midShootPosShotNoStop", rc.sequenceFactory.getToShooterStateCommand(6000, 14));
        NamedCommands.registerCommand("botShootPosShotNoStop", rc.sequenceFactory.getToShooterStateCommand(5500, 14));
        NamedCommands.registerCommand("visionShot", new ParallelCommandGroup(
            new VisionTurnToAprilTag(()->0, ()->0, ()->0, rc.shooterVision, rc.chassis, rc.navx).withTimeout(1),
            new ShooterSetVision(rc.shooter, rc.shooterVision, rc.shooterFlywheel)
        ));

        NamedCommands.registerCommand("rampUpVision", new RunCommand(()->rc.shooterFlywheel.setRPM(4000)));

        NamedCommands.registerCommand("setDownShooter", rc.sequenceFactory.getStopShooterCommand());

        // SmartDashboard.putData("Field", pathPlannerField);

        // PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
        //     pathPlannerField.setRobotPose(pose);
        // });

        // PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
        //     pathPlannerField.getObject("target pose").setPose(pose);
        // });

        // PathPlannerLogging.setLogActivePathCallback((list)->pathPlannerField.getObject("path").setPoses(list));

    }

    Command pathPlannerFollowPathManual(String pathName){
        // PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(pathName);
        
        return AutoBuilder.followPath(path);
    }

    Command makePathFindToPoseCommand(Pose2d pose){
        if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red){
            pose = new Pose2d(16.542-pose.getX(), pose.getY(), new Rotation2d(Math.PI-pose.getRotation().getRadians())); 
        }
        
        PathConstraints constraints = new PathConstraints(
        3, 4,
        8.4, 8.1);

        return AutoBuilder.pathfindToPose(pose, constraints);
    }

    Command makePathFindThenFollowPathCommand(PathPlannerPath path){
        PathConstraints constraints = new PathConstraints(
        3.5, 5.1,
        8.4, 8.1);

        return AutoBuilder.pathfindThenFollowPath(path, constraints);
    }

    public Command getVisionPathFindCommand(Pose2d pose, double flywheelRpm, double shooterAngle){
        return new ParallelDeadlineGroup(
                    makePathFindToPoseCommand(pose),
                    //spinup before we see target in case w e see last second
                    new RunCommand(()->rc.sequenceFactory.getToShooterStateCommand(flywheelRpm, shooterAngle)).until(rc.shooterVision::hasValidTarget)
                        .andThen(new ShooterSetVision(rc.shooter, rc.shooterVision, rc.shooterFlywheel).runForever()),
                    new PassthroughAlignNote(rc.passthrough, rc.intake)
                );
    }

    public Command ExampleAuto(){
        return new InstantCommand(()->{},rc.chassis,rc.climber);
    }

    //Not overloaded correctly, whichever one works i'll keep
    public Command makeVisionAuto(PathPlannerPath path, PathPlannerPath returnPath, double delay, double targetAngle, double targetRPM){
        Command sequence = 
        new SequentialCommandGroup(
            new InstantCommand(()->rc.shooterVision.enableAutoVision(false)),
            new InstantCommand(()->rc.intakeVision.setPipeline(IntakePipeline.kNote)),
            //----------------------------FOLLOW PATH STOP WHEN SEE NOTE----------------------------
            new ParallelDeadlineGroup(
                AutoBuilder.followPath(path).until(()->Math.abs(rc.swerveDrivePoseEstimator.getEstimatedPosition().getX()-(16.542/2)) < ((16.542/2)-5.474)&&rc.intakeVision.hasValidTarget()),
                new ParallelCommandGroup(
                    new ShooterSetVision(rc.shooter, rc.shooterVision, rc.shooterFlywheel, targetAngle, targetRPM),
                    new SequentialCommandGroup(
                        new WaitCommand(delay),
                        new ParallelCommandGroup(
                            new RunCommand(rc.intake::intake, rc.intake).finallyDo(rc.intake::stop), 
                            new RunCommand(rc.passthrough::intake, rc.passthrough).finallyDo(rc.passthrough::stop)
                        )
                        .until(()->!rc.passthrough.isBlocked()&&!rc.intake.isBlocked())
                    )
                )
            ),
            //What if intake vision fails? will the next part simply move forward with no note to track?

            //------------------------------TRACK NOTE UNTIL HAVE NOTE------------------------------
            new VisionTrackNoteAuto(()->0.3, ()->0.0, ()->0.0, rc.chassis, rc.intake, rc.passthrough, rc.intakeVision, rc.leds)
                .withTimeout(intakeVisionAutoTimeout),
            //What if we follow opponent cause also red? Do we need google coral then?
            //We simply have to keep on moving forward, even if we might cross lin, else if we lose track for a moment we just stop

            //------------------------PATHFIND, SPIN UP UNTIL SEE TAG, VISION TO SPEAKER------------
            new ParallelDeadlineGroup(
                makePathFindThenFollowPathCommand(returnPath),
                new ShooterSetVision(rc.shooter, rc.shooterVision, rc.shooterFlywheel, 14, 6000).runForever(),
                new PassthroughAlignNote(rc.passthrough, rc.intake),
                new InstantCommand(rc.leds::pink)
            ),

            //-----------------------------------FIX SHOOTING ANGLES--------------------------------
            new InstantCommand(()->rc.shooterVision.enableAutoVision(true)), //Now that we are somewhat stable, update odo
            new ParallelDeadlineGroup(
                new ShooterSetVision(rc.shooter, rc.shooterVision, rc.shooterFlywheel),
                new VisionTurnToSpeakerOpticalOnly(()->0.0,()->0.0,()->0.0,rc.shooterVision, rc.chassis, rc.navx)
                //     .until(() -> Math.abs(rc.shooterVision.getVisibleTargetData().orElse(defaultLimelightReadings).angleHorizontal)<5)
            ).withTimeout(shooterVisionAutoTimeout),

            //-------------------------------------------SHOOT---------------------------------------
            new InstantCommand(()->rc.shooterVision.enableAutoVision(false)),
            new ParallelCommandGroup(
                new RunCommand(rc.intake::intake, rc.intake).finallyDo(rc.intake::stop), 
                new RunCommand(rc.passthrough::intake, rc.passthrough).finallyDo(rc.passthrough::stop)
            )
            .until(()->!rc.passthrough.isBlocked()&&!rc.intake.isBlocked())
        );

        return sequence;
    }

    //Not overloaded correctly, whichever one works i'll keep
    public Command makeVisionAuto(PathPlannerPath path, PathPlannerPath returnPath){
        Command sequence = 
        new SequentialCommandGroup(
            new InstantCommand(()->rc.shooterVision.enableAutoVision(false)),
            new InstantCommand(()->rc.intakeVision.setPipeline(IntakePipeline.kNote)),
            //----------------------------FOLLOW PATH STOP WHEN SEE NOTE----------------------------
            AutoBuilder.followPath(path).until(()->Math.abs(rc.swerveDrivePoseEstimator.getEstimatedPosition().getX()-(16.542/2)) < ((16.542/2)-5.474)&&rc.intakeVision.hasValidTarget()),
            //What if intake vision fails? will the next part simply move forward with no note to track?

            //------------------------------TRACK NOTE UNTIL HAVE NOTE------------------------------
            new VisionTrackNoteAuto(()->0.4, ()->0.0, ()->0.0, rc.chassis, rc.intake, rc.passthrough, rc.intakeVision, rc.leds)
                .withTimeout(intakeVisionAutoTimeout),
            //What if we follow opponent cause also red? Do we need google coral then?
            //We simply have to keep on moving forward, even if we might cross lin, else if we lose track for a moment we just stop

            //------------------------PATHFIND, SPIN UP UNTIL SEE TAG, VISION TO SPEAKER------------
            new ParallelDeadlineGroup(
                makePathFindThenFollowPathCommand(returnPath),
                new ShooterSetVision(rc.shooter, rc.shooterVision, rc.shooterFlywheel, 40, 5500).runForever(),
                new PassthroughAlignNote(rc.passthrough, rc.intake),
                new InstantCommand(rc.leds::pink)
            ),

            //-----------------------------------FIX SHOOTING ANGLES--------------------------------
            new InstantCommand(()->rc.shooterVision.enableAutoVision(true)), //Now that we are somewhat stable, update odo
            new ParallelDeadlineGroup(
                new ShooterSetVision(rc.shooter, rc.shooterVision, rc.shooterFlywheel),
                // new VisionTurnToSpeakerOpticalOnly(()->0.0,()->0.0,()->0.0,rc.shooterVision, rc.chassis, rc.navx)
                //     .until(() -> Math.abs(rc.shooterVision.getVisibleTargetData().orElse(defaultLimelightReadings).angleHorizontal)<5)
                new RunCommand(()->rc.chassis.drive(0, 0, 0, false, false), rc.chassis)
            ).withTimeout(shooterVisionAutoTimeout),

            //-------------------------------------------SHOOT---------------------------------------
            new InstantCommand(()->rc.shooterVision.enableAutoVision(false)),
            new ParallelCommandGroup(
                new RunCommand(rc.intake::intake, rc.intake).finallyDo(rc.intake::stop), 
                new RunCommand(rc.passthrough::intake, rc.passthrough).finallyDo(rc.passthrough::stop)
            )
            .until(()->!rc.passthrough.isBlocked()&&!rc.intake.isBlocked())
        );

        return sequence;
    }

    public Command makeVisionAuto(PathPlannerPath path, Pose2d endPose){
        ShooterVision.LimelightReadings defaultLimelightReadings = rc.shooterVision.getDefaultLimelightReadings();
        
        Command sequence = 
        new SequentialCommandGroup(
            new InstantCommand(()->rc.shooterVision.enableAutoVision(false)),
            new InstantCommand(()->rc.intakeVision.setPipeline(IntakePipeline.kNote)),
            //----------------------------FOLLOW PATH STOP WHEN SEE NOTE----------------------------
            AutoBuilder.followPath(path).until(()->rc.intakeVision.hasValidTarget()),
            //What if intake vision fails? will the next part simply move forward with no note to track?

            //------------------------------TRACK NOTE UNTIL HAVE NOTE------------------------------
            new VisionTrackNoteAuto(()->0.25, ()->0.0, ()->0.0, rc.chassis, rc.intake, rc.passthrough, rc.intakeVision, rc.leds)
                .withTimeout(intakeVisionAutoTimeout),
            //What if we follow opponent cause also red? Do we need google coral then?
            //We simply have to keep on moving forward, even if we might cross lin, else if we lose track for a moment we just stop

            //------------------------PATHFIND, SPIN UP UNTIL SEE TAG, VISION TO SPEAKER------------
            new ParallelDeadlineGroup(
                makePathFindToPoseCommand(endPose),
                new ShooterSetVision(rc.shooter, rc.shooterVision, rc.shooterFlywheel, 40, 5500).runForever(),
                new PassthroughAlignNote(rc.passthrough, rc.intake),
                new InstantCommand(rc.leds::pink)
            ),

            //-----------------------------------FIX SHOOTING ANGLES--------------------------------
            new InstantCommand(()->rc.shooterVision.enableAutoVision(true)), //Now that we are somewhat stable, update odo
            new ParallelDeadlineGroup(
                new ShooterSetVision(rc.shooter, rc.shooterVision, rc.shooterFlywheel),
                new VisionTurnToSpeakerOpticalOnly(()->0.0,()->0.0,()->0.0,rc.shooterVision, rc.chassis, rc.navx)
            ).withTimeout(shooterVisionAutoTimeout),

            //-------------------------------------------SHOOT---------------------------------------
            new ParallelCommandGroup(
                new RunCommand(rc.intake::intake, rc.intake).finallyDo(rc.intake::stop), 
                new RunCommand(rc.passthrough::intake, rc.passthrough).finallyDo(rc.passthrough::stop)
            )
            .until(()->!rc.passthrough.isBlocked()&&!rc.intake.isBlocked())
        );

        return sequence;
    }

    public SendableChooser<Command> getAutoChooser(){
        try{
            initAutoChooserFuture.get();
       }
       catch(Exception e){
           //If the auto cannot build, then we get an error and print it.
           System.err.println("Failed to build auto command ");
           System.err.println(e);
       }

        return autoChooser;
    }

}
