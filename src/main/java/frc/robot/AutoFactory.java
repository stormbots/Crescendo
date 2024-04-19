// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Future;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

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
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.ChassisConstants.AutoConstants;
import frc.robot.ChassisConstants.DriveConstants;
import frc.robot.commands.PassthroughAlignNote;
import frc.robot.commands.SetDunkArmSlew;
import frc.robot.commands.ShooterSetVision;
import frc.robot.commands.VisionTrackNoteAuto;
import frc.robot.commands.VisionTurnToAprilTag;
import frc.robot.subsystems.PassthroughLock;
import frc.robot.subsystems.Shooter;




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

    public AutoFactory(RobotContainer rc){
        this.rc = rc;
        initPathPlannerStuff();

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        
        // initAutoChooser(); //aktes too long, run in background
        initAutoChooserFuture = CompletableFuture.supplyAsync(this::initAutoChooser);

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    boolean initAutoChooser(){
        BooleanSupplier isBlue = () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Blue;
            }
            return false;
        };

        autoChooser.setDefaultOption("Please Select Auto", new InstantCommand());

        // autoChooser.addOption("vv Test Autos vv", new InstantCommand());


        // autoChooser.addOption("Two Meter Auto Path Planner", new InstantCommand(()->rc.chassis.resetOdometry(new Pose2d(0,0, new Rotation2d()))).andThen(pathPlannerFollowPathManual("twoMeterAuto")));

        // autoChooser.addOption("auto Trap", new InstantCommand()
        //     .andThen(()->rc.chassis.setFieldCentricOffset(60))
        //     .andThen(()->rc.chassis.resetOdometry(new Pose2d(4.9, 4.1, new Rotation2d(Units.Degrees.of(-60)))))
        //     .andThen(rc.sequenceFactory.getTrapSequenceCommand(pathPlannerFollowPathManual("ampTrap"), Units.Degrees.of(-60)))
        // );

        autoChooser.addOption("5NoteAmpChoreo", new InstantCommand()
            .andThen(()->rc.chassis.setFieldCentricOffset(-60, isBlue))
            .andThen(new PathPlannerAuto("ChoreoAmpAuto"))
        );

        // autoChooser.addOption("4NoteAmpRushChoreo", new InstantCommand()
        //     .andThen(()->rc.chassis.setFieldCentricOffset(0, isBlue))
        //     .andThen(new PathPlannerAuto("FarStartChoreoAmpRushAuto"))
        // );

        autoChooser.addOption("4NoteSourceRushChoreo", new InstantCommand()
            .andThen(()->rc.chassis.setFieldCentricOffset(0, isBlue))
            .andThen(new PathPlannerAuto("ChoreoSourceRushAuto"))
        );
        autoChooser.addOption("FarStart4NoteSourceRushChoreo", new InstantCommand()
            .andThen(()->rc.chassis.setFieldCentricOffset(0, isBlue))
            .andThen(new PathPlannerAuto("FarStartChoreoSourceRushAuto"))
        );

        autoChooser.addOption("5NoteCenterTopFirstChoreo", new InstantCommand()
            .andThen(()->rc.chassis.setFieldCentricOffset(0, isBlue))
            .andThen(new PathPlannerAuto("ChoreoCenterTopFirstAuto"))
        );
        
        // autoChooser.addOption("TestChoreoAuto", new InstantCommand()
        //     .andThen(()->rc.chassis.setFieldCentricOffset(0, isBlue))
        //     .andThen(new PathPlannerAuto("TestChoreoAuto"))
        // );

        autoChooser.addOption("vv SysID vv", new InstantCommand());
        
        autoChooser.addOption("quasi forward", rc.chassis.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("quasi backward", rc.chassis.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("dynamic forward", rc.chassis.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("dynamic backward", rc.chassis.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        autoChooser.addOption("sysid Flywheel routine", 
            new InstantCommand()
                .andThen(rc.shooterFlywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward).withTimeout(5))
                .andThen(new WaitCommand(5))
                .andThen(rc.shooterFlywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).withTimeout(5))
                .andThen(new WaitCommand(5))
                .andThen(rc.shooterFlywheel.sysIdDynamic(SysIdRoutine.Direction.kForward).withTimeout(2))
                .andThen(new WaitCommand(5))
                .andThen(rc.shooterFlywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse).withTimeout(2))
                .andThen(new WaitCommand(5))

        );

        autoChooser.addOption("sysid shooter routine", 
            new InstantCommand()
                .andThen(rc.shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward).withTimeout(2))
                .andThen(new WaitCommand(2))
                .andThen(rc.shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).withTimeout(2))
                .andThen(new WaitCommand(2))
                .andThen(rc.shooter.sysIdDynamic(SysIdRoutine.Direction.kForward).withTimeout(1))
                .andThen(new WaitCommand(2))
                .andThen(rc.shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse).withTimeout(1))
                .andThen(new WaitCommand(2))

        );

        autoChooser.addOption("sysid dunkArm routine", 
            new InstantCommand()
                .andThen(rc.dunkArm.sysIdQuasistatic(SysIdRoutine.Direction.kForward).withTimeout(2))
                .andThen(new WaitCommand(2))
                .andThen(rc.dunkArm.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).withTimeout(2))
                .andThen(new WaitCommand(2))
                .andThen(rc.dunkArm.sysIdDynamic(SysIdRoutine.Direction.kForward).withTimeout(2))
                .andThen(new WaitCommand(2))
                .andThen(rc.dunkArm.sysIdDynamic(SysIdRoutine.Direction.kReverse).withTimeout(2))
                .andThen(new WaitCommand(2))

        );

        //absolute pain in the butt to read and debug
        autoChooser.addOption("ampGamePieceVisionChoreo", 
            new InstantCommand(()->rc.chassis.setFieldCentricOffset(-60, isBlue))
            .andThen(new InstantCommand(()->rc.chassis.resetOdometryAllianceManaged(new Pose2d(0.747,6.659,new Rotation2d(1.052)))))
            .andThen(new InstantCommand(()->rc.shooterVision.enableAutoVision(true)))
            //Probably a better way to do what is shown below
            .andThen(
                new ParallelRaceGroup(
                    pathPlannerFollowPathManual("AmpRunThrough.1"),
                    new WaitCommand(1.6)
                        .andThen(
                            new RunCommand(()->{}).until(()->rc.intakeVision.hasValidTarget())
                        )
                )
            )
            //main issue is 1. we dont find a note 2.opponent takes note and we just keep driving at 0.2 of max
            .andThen(
                new VisionTrackNoteAuto(()->0.5, ()->0.0, ()->0.0, rc.chassis, rc.intake, rc.passthrough, rc.intakeVision, rc.leds).withTimeout(2) //Find a better value than 0.2
            )
            .andThen(
                getVisionPathFindCommand(new Pose2d(4.15,6.33,new Rotation2d(0.185)), 6000, 14)
            )
            .andThen(
                rc.sequenceFactory.getVisionAlignmentShotCommand().withTimeout(0.5)
            )
            .andThen(
                rc.sequenceFactory.getIntakeShootCommand().withTimeout(0.4)
            )
            .andThen(
                pathPlannerFollowPathManual("TopShootTopMidShare.1").until(()->rc.intakeVision.hasValidTarget())
            )
            .andThen(
                new VisionTrackNoteAuto(()->0.5, ()->0.0, ()->0.0, rc.chassis, rc.intake, rc.passthrough, rc.intakeVision, rc.leds).withTimeout(3) //Find a better value than 0.2
            )
            .andThen(
                getVisionPathFindCommand(new Pose2d(4.15,6.33,new Rotation2d(0.185)), 6000, 14)
            )
            .andThen(
                rc.sequenceFactory.getVisionAlignmentShotCommand().withTimeout(0.5)
            )
            .andThen(
                rc.sequenceFactory.getIntakeShootCommand().withTimeout(0.4)
            )
            .andThen(
                pathPlannerFollowPathManual("TopShootMidShare.1")
                .andThen(pathPlannerFollowPathManual("TopShootMidShare.2"))
                .andThen(new WaitCommand(10))
                //makes sure if we dont see any notes that we just stay on that edge.
                .until(()->rc.intakeVision.hasValidTarget())
            )
            .andThen(
                new VisionTrackNoteAuto(()->0.5, ()->0.0, ()->0.0, rc.chassis, rc.intake, rc.passthrough, rc.intakeVision, rc.leds).withTimeout(3) //Find a better value than 0.2
            )
            .andThen(
                getVisionPathFindCommand(new Pose2d(4.15,6.33,new Rotation2d(0.185)), 6000, 14)
            )
            .andThen(
                rc.sequenceFactory.getVisionAlignmentShotCommand().withTimeout(0.5)
            )
            .andThen(
                rc.sequenceFactory.getIntakeShootCommand().withTimeout(0.4)
            )
            
        );

        //absolute pain in the butt to read and debug
        autoChooser.addOption("sourceGamePieceVisionChoreo", 
            new InstantCommand(()->rc.chassis.setFieldCentricOffset(0, isBlue))
            .andThen(new InstantCommand(()->rc.chassis.resetOdometryAllianceManaged(new Pose2d(1.5,3.589,new Rotation2d()))))
            .andThen(new InstantCommand(()->rc.shooterVision.enableAutoVision(true)))
            //Probably a better way to do what is shown below
            .andThen(
                new ParallelRaceGroup(
                    pathPlannerFollowPathManual("FarStartSourceRunThrough.1"),
                    new WaitCommand(1.0)
                        .andThen(
                            new RunCommand(()->{}).until(()->rc.intakeVision.hasValidTarget())
                        )
                )
            )
            .andThen(
                new VisionTrackNoteAuto(()->0.5, ()->0.0, ()->0.0, rc.chassis, rc.intake, rc.passthrough, rc.intakeVision, rc.leds).withTimeout(3) //Find a better value than 0.2
            )
            .andThen(
                // getVisionPathFindCommand(new Pose2d(3.047,2.791,new Rotation2d(-0.774)), 6000, 14)
                new ParallelDeadlineGroup(
                    makePathFindToPoseCommand(new Pose2d(3.047,2.791,new Rotation2d(-0.774))),
                    new RunCommand(()->rc.sequenceFactory.getToShooterStateCommand(6000, 14)).until(rc.shooterVision::hasValidTarget)
                        .andThen(new ShooterSetVision(rc.shooter, rc.shooterVision, rc.shooterFlywheel).runForever()),
                    new PassthroughAlignNote(rc.passthrough, rc.intake),
                    new InstantCommand(rc.leds::preparing)
                )
            )
            .andThen(
                rc.sequenceFactory.getVisionAlignmentShotCommand().withTimeout(0.5)
            )
            .andThen(
                rc.sequenceFactory.getIntakeShootCommand().withTimeout(0.4)
            )
            .andThen(
                pathPlannerFollowPathManual("BotShootBotMidShare.1").until(()->rc.intakeVision.hasValidTarget())
            )
            .andThen(
                new VisionTrackNoteAuto(()->0.5, ()->0.0, ()->0.0, rc.chassis, rc.intake, rc.passthrough, rc.intakeVision, rc.leds)
                .withTimeout(3)
            )
            .andThen(
                // getVisionPathFindCommand(new Pose2d(3.047,2.791,new Rotation2d(-0.774)), 6000, 14)
                new ParallelDeadlineGroup(
                    makePathFindToPoseCommand(new Pose2d(3.047,2.791,new Rotation2d(-0.774))),
                    new RunCommand(()->rc.sequenceFactory.getToShooterStateCommand(6000, 14)).until(rc.shooterVision::hasValidTarget)
                        .andThen(new ShooterSetVision(rc.shooter, rc.shooterVision, rc.shooterFlywheel).runForever()),
                    new PassthroughAlignNote(rc.passthrough, rc.intake),
                    new InstantCommand(rc.leds::preparing)
                )
            )
            .andThen(
                rc.sequenceFactory.getVisionAlignmentShotCommand().withTimeout(0.5)
            )
            .andThen(
                rc.sequenceFactory.getIntakeShootCommand().withTimeout(0.4)
            )
            .andThen(
                pathPlannerFollowPathManual("BotShootMidShare.1")
                .andThen(new WaitCommand(10))
                //makes sure if we dont see any notes that we just stay on that edge.
                .until(()->rc.intakeVision.hasValidTarget())
            )
            .andThen(
                new VisionTrackNoteAuto(()->0.5, ()->0.0, ()->0.0, rc.chassis, rc.intake, rc.passthrough, rc.intakeVision, rc.leds)
                .withTimeout(3)
            )
            .andThen(
                // getVisionPathFindCommand(new Pose2d(3.047,2.791,new Rotation2d(-0.774)), 6000, 14)
                new ParallelDeadlineGroup(
                    makePathFindToPoseCommand(new Pose2d(3.047,2.791,new Rotation2d(-0.774))),
                    new RunCommand(()->rc.sequenceFactory.getToShooterStateCommand(6000, 14)).until(rc.shooterVision::hasValidTarget)
                        .andThen(new ShooterSetVision(rc.shooter, rc.shooterVision, rc.shooterFlywheel).runForever()),
                    new PassthroughAlignNote(rc.passthrough, rc.intake),
                    new InstantCommand(rc.leds::preparing)
                )
            )
            .andThen(
                rc.sequenceFactory.getVisionAlignmentShotCommand().withTimeout(0.5)
            )
            .andThen(
                rc.sequenceFactory.getIntakeShootCommand().withTimeout(0.4)
            )
            
        );

        return false; //will bryan cry tonight 
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

        //this kinda sucks but its better than guessing values for now, fix code struct later
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
        NamedCommands.registerCommand("topNoteShotNoStopOnTheRun", rc.sequenceFactory.getToShooterStateCommand(5000, 20.7));

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

        SmartDashboard.putData("Field", pathPlannerField);

        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            pathPlannerField.setRobotPose(pose);
        });

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            pathPlannerField.getObject("target pose").setPose(pose);
        });

        PathPlannerLogging.setLogActivePathCallback((list)->pathPlannerField.getObject("path").setPoses(list));

    }

    Command pathPlannerFollowPathManual(String pathName){
        // PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory(pathName);
        
        return AutoBuilder.followPath(path);
    }
    
    //NOTE: INITIALIZES ONCE ON STARTUP, DETERMINES SIDE BEFOREHAND
    Command makePathFindToPoseCommand(Pose2d pose){
        if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red){
            SmartDashboard.putBoolean("PathfindBoolean", DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red);
            pose = new Pose2d(16.542-pose.getX(), pose.getY(), new Rotation2d(Math.PI-pose.getRotation().getRadians())); 
        }
        
        PathConstraints constraints = new PathConstraints(
        3.5, 5.1,
        8.4, 8.1);

        return AutoBuilder.pathfindToPose(pose, constraints);
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
