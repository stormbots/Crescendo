// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.ChassisConstants.AutoConstants;
import frc.robot.ChassisConstants.DriveConstants;
import frc.robot.commands.SetDunkArmSlew;
import frc.robot.commands.ShooterSetVision;
import frc.robot.commands.VisionTurnToAprilTag;
import frc.robot.subsystems.DunkArm;
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

        
        // initAutoChooser(); //This for somereason had to be made for the autos to deploy
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

        autoChooser.addOption("untested autos", new InstantCommand());

        autoChooser.addOption("vv Test Autos vv", new InstantCommand());

        autoChooser.addOption("rpmandShoot", rc.sequenceFactory.getSetRPMandShootCommand(3000, 20));



        // autoChooser.addOption("Two Meter Auto Path Planner", new InstantCommand(()->rc.chassis.resetOdometry(new Pose2d(0,0, new Rotation2d()))).andThen(pathPlannerFollowPathManual("twoMeterAuto")));
        // autoChooser.addOption("Rotate + Two Meter Auto Path Planner", pathPlannerFollowPathManual("twoMeterRotation"));

        // autoChooser.addOption("auto Trap", new InstantCommand()
        //     .andThen(()->rc.chassis.setFieldCentricOffset(60))
        //     .andThen(()->rc.chassis.resetOdometry(new Pose2d(4.9, 4.1, new Rotation2d(Units.Degrees.of(-60)))))
        //     .andThen(rc.sequenceFactory.getTrapSequenceCommand(pathPlannerFollowPathManual("ampTrap"), Units.Degrees.of(-60)))
        // );

        // autoChooser.addOption("twoMeterRotation", new InstantCommand()
        //     .andThen(()->rc.chassis.setFieldCentricOffset(0))
        //     .andThen(()->rc.chassis.resetOdometry(new Pose2d(0,0,new Rotation2d())))
        //     .andThen(pathPlannerFollowPathManual("twoMeterRotation"))
        //     );
        
        // autoChooser.addOption("resetCenterPositionRed", new InstantCommand()
        //     .andThen(()->rc.chassis.setFieldCentricOffset(0))
        //     .andThen(()->rc.chassis.resetOdometry(new Pose2d(16.6-1.3, 5.65, new Rotation2d(Units.Degrees.of(180)))))
        // );

        autoChooser.addOption("vv PathPlanner Untested Autos vv", new InstantCommand());

        // autoChooser.addOption("basicAmpAuto", 
        // new InstantCommand()
        // .andThen(()->rc.chassis.setFieldCentricOffset(-60))
        // .andThen(new PathPlannerAuto("basicAmpAuto")));

        autoChooser.addOption("5NoteAmp", new InstantCommand()
            .andThen(()->rc.chassis.setFieldCentricOffset(-60, isBlue))
            .andThen(new PathPlannerAuto("5NoteAmpAuto"))
        );

        // autoChooser.addOption("4noteAmp", new InstantCommand()
        //     .andThen(()->rc.chassis.setFieldCentricOffset(-60, isBlue))
        //     .andThen(new PathPlannerAuto("4NoteAmpAuto"))
        // );

        autoChooser.addOption("4NoteSource", new InstantCommand()
            .andThen(()->rc.chassis.setFieldCentricOffset(60, isBlue))
            .andThen(new PathPlannerAuto("4NoteSourceAuto"))
        );

        autoChooser.addOption("5NoteCenter", new InstantCommand()
            .andThen(()->rc.chassis.setFieldCentricOffset(0, isBlue))
            .andThen(new PathPlannerAuto("5NoteCenterAuto"))
        );

        // autoChooser.addOption("Vision5NoteAmp", new InstantCommand()
        //     .andThen(()->rc.chassis.setFieldCentricOffset(-60, isBlue))
        //     .andThen(new PathPlannerAuto("4NoteAmpAutoVision"))
        // );

        // autoChooser.addOption("4NoteCenter", new InstantCommand()
        //     .andThen(()->rc.chassis.setFieldCentricOffset(0, isBlue))
        //     .andThen(new PathPlannerAuto("4NoteCenterAuto"))
        // );

        // autoChooser.addOption("closeNote", new InstantCommand()
        //     .andThen(()->rc.chassis.setFieldCentricOffset(60, isBlue))
        //     .andThen(new PathPlannerAuto("closeNoteAuto"))
        // );

        // autoChooser.addOption("2NoteAmp", new InstantCommand()
        // .andThen(()->rc.chassis.setFieldCentricOffset(-60, isBlue))
        // .andThen(new PathPlannerAuto("2NoteAmpAuto")));

        // autoChooser.addOption("2NoteSource", new InstantCommand()
        // .andThen(()->rc.chassis.setFieldCentricOffset(60, isBlue))
        // .andThen(new PathPlannerAuto("2NoteSourceAuto")));

        autoChooser.addOption("TwoMeter", new InstantCommand()
            .andThen(()->rc.chassis.setFieldCentricOffset(0))
            .andThen(new PathPlannerAuto("2MeterAuto")));

        autoChooser.addOption("Circle", new InstantCommand()
            .andThen(()->rc.chassis.setFieldCentricOffset(90, isBlue))
            .andThen(new PathPlannerAuto("CircleAuto")));

        autoChooser.addOption("5NoteChoreo", new InstantCommand()
            .andThen(()->rc.chassis.setFieldCentricOffset(-60, isBlue))
            .andThen(new PathPlannerAuto("ChoreoAmpAuto"))
        );

        autoChooser.addOption("5NoteChoreo", new InstantCommand()
            .andThen(()->rc.chassis.setFieldCentricOffset(-60, isBlue))
            .andThen(new PathPlannerAuto("ChoreoAmpAuto"))
        );

        autoChooser.addOption("5NoteChoreo", new InstantCommand()
            .andThen(()->rc.chassis.setFieldCentricOffset(-60, isBlue))
            .andThen(new PathPlannerAuto("ChoreoAmpAuto"))
        );

        autoChooser.addOption("vv SysID vv", new InstantCommand());
        
        autoChooser.addOption("quasi forward", rc.chassis.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("quasi backward", rc.chassis.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("dynamic forward", rc.chassis.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption("dynamic backward", rc.chassis.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        autoChooser.addOption("sysid Flywheel routine", 
            new InstantCommand()
                .andThen(rc.shooterFlywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward).withTimeout(5))
                .andThen(new WaitCommand(2))
                .andThen(rc.shooterFlywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).withTimeout(5))
                .andThen(new WaitCommand(2))
                .andThen(rc.shooterFlywheel.sysIdDynamic(SysIdRoutine.Direction.kForward).withTimeout(2))
                .andThen(new WaitCommand(4))
                .andThen(rc.shooterFlywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse).withTimeout(2))
                .andThen(new WaitCommand(4))

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
            new PIDConstants(5*0.5), 
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
        LUT shooterLUT = Shooter.lut;
        Measure<Distance> distance = Units.Meters.of(Math.hypot(4.3-0.4, 6.25-5.55));
        
        var angle = shooterLUT.get(distance.in(Units.Inches))[0];
        var rpm = shooterLUT.get(distance.in(Units.Inches))[1];


        

        NamedCommands.registerCommand("intakeAndAlign", rc.sequenceFactory.getIntakeThenAlignCommand());
        NamedCommands.registerCommand("intakeFull", new ParallelCommandGroup(new RunCommand(rc.intake::intake, rc.intake), new RunCommand(rc.passthrough::intake, rc.passthrough)));
        NamedCommands.registerCommand("intakeShoot", new ParallelCommandGroup(new RunCommand(rc.intake::intake, rc.intake), new RunCommand(rc.passthrough::intake, rc.passthrough)).until(()->!rc.passthrough.isBlocked()&&!rc.intake.isBlocked()));
        NamedCommands.registerCommand("intakeStop", rc.sequenceFactory.getStopIntakingCommand().withTimeout(5));

        NamedCommands.registerCommand("subwooferShot", rc.sequenceFactory.getSetRPMandShootCommand(5500, 45));
        NamedCommands.registerCommand("spinUpShotNoStop", rc.sequenceFactory.getToShooterStateCommand(7000, 32.5));
        NamedCommands.registerCommand("stageShot", rc.sequenceFactory.getSetRPMandShootCommand(8000, 20));
        NamedCommands.registerCommand("stageShotNoStop", rc.sequenceFactory.getToShooterStateCommand(8000, 20));
        NamedCommands.registerCommand("stopFlywheel", new WaitCommand(0.1).andThen(new InstantCommand(()->rc.shooterFlywheel.setRPM(0))));
        NamedCommands.registerCommand("dunkArmUp", new SetDunkArmSlew(80, rc.dunkArm));
        NamedCommands.registerCommand("dunkArmDown", new SetDunkArmSlew(-25, rc.dunkArm));

        // NamedCommands.registerCommand("topSpinUpShotNoStop", new ShooterSetOdometry(rc.shooter, rc.shooterFlywheel, new Pose2d(2.15, 6.5, new Rotation2d())).runForever());
        // NamedCommands.registerCommand("midSpinUpShotNoStop", new ShooterSetOdometry(rc.shooter, rc.shooterFlywheel, new Pose2d(2.15, 6.5, new Rotation2d())).runForever());
        // NamedCommands.registerCommand("botSpinUpShotNoStop", new ShooterSetOdometry(rc.shooter, rc.shooterFlywheel, new Pose2d(2.15, 6.5, new Rotation2d())).runForever());

        // NamedCommands.registerCommand("topNoteShotNoStop", new ShooterSetOdometry(rc.shooter, rc.shooterFlywheel, new Pose2d(2.9, 7, new Rotation2d())).runForever());
        // NamedCommands.registerCommand("midNoteShotNoStop", new ShooterSetOdometry(rc.shooter, rc.shooterFlywheel, new Pose2d(2.9, 5.55, new Rotation2d())).runForever());
        // NamedCommands.registerCommand("botNoteShotNoStop", new ShooterSetOdometry(rc.shooter, rc.shooterFlywheel, new Pose2d(2.78, 4.9, new Rotation2d())).runForever());

        // NamedCommands.registerCommand("topShootPosShotNoStop", new ShooterSetOdometry(rc.shooter, rc.shooterFlywheel, new Pose2d(4.3, 6.25, new Rotation2d())).runForever());
        // NamedCommands.registerCommand("midShootPosShotNoStop", new ShooterSetOdometry(rc.shooter, rc.shooterFlywheel, new Pose2d(4.3, 4.85, new Rotation2d())).runForever());
        // NamedCommands.registerCommand("botShootPosShotNoStop", new ShooterSetOdometry(rc.shooter, rc.shooterFlywheel, new Pose2d(3, 2.8, new Rotation2d())).runForever());

        NamedCommands.registerCommand("topSpinUpShotNoStop", rc.sequenceFactory.getToShooterStateCommand(4000, 28.5-0.5));
        NamedCommands.registerCommand("midSpinUpShotNoStop", rc.sequenceFactory.getToShooterStateCommand(1000, 32.5));
        NamedCommands.registerCommand("botSpinUpShotNoStop", rc.sequenceFactory.getToShooterStateCommand(4000, 28));

        NamedCommands.registerCommand("topNoteShotNoStop", rc.sequenceFactory.getToShooterStateCommand(5000, 20.7));
        NamedCommands.registerCommand("midNoteShotNoStop", rc.sequenceFactory.getToShooterStateCommand(1000, 32.5));
        NamedCommands.registerCommand("botNoteShotNoStop", rc.sequenceFactory.getToShooterStateCommand(5000, 20.5));

        NamedCommands.registerCommand("topShootPosShotNoStop", rc.sequenceFactory.getToShooterStateCommand(5500, 14));
        NamedCommands.registerCommand("topShootPosShotNoStop3", rc.sequenceFactory.getToShooterStateCommand(5500, 14));
        NamedCommands.registerCommand("topShootPosShotNoStop4", rc.sequenceFactory.getToShooterStateCommand(5500, 14));
        NamedCommands.registerCommand("midShootPosShotNoStop", rc.sequenceFactory.getToShooterStateCommand(6000, 15));
        NamedCommands.registerCommand("botShootPosShotNoStop", rc.sequenceFactory.getToShooterStateCommand(6000, 14));
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
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        
        return AutoBuilder.followPath(path);
    }
    
    Command makePathFindToPoseCommand(Pose2d pose){
        PathConstraints constraints = new PathConstraints(
        1.0, 1.0,
        Math.PI, Math.PI);

        return AutoBuilder.pathfindToPose(pose, constraints);
    }

    public Command ExampleAuto(){
        return new InstantCommand(()->{},rc.chassis,rc.climber);
    }


    SwerveControllerCommand generateSwerveControllerCommand(Trajectory trajectory){
    
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

    public Command makeBasicAuto(Pose2d start, List<Translation2d> midpoints,Pose2d end, double rpm, double angle){
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
        .andThen(rc.sequenceFactory.getSetRPMandShootCommand(rpm, angle))
        .andThen(new ParallelDeadlineGroup(
            generateSwerveControllerCommand(start, midpoints, end).andThen(new WaitCommand(1)).withTimeout(6),
            rc.sequenceFactory.getIntakeThenAlignCommand()
        ))
        ;
    }

    public Command makeBasicAuto(Pose2d start, List<Translation2d> midpoints,Pose2d end){
        return makeBasicAuto(start, midpoints, end, 5500, 47);

    }

    public Command makeBasicAuto(Pose2d start, Pose2d end){
        return makeBasicAuto(start, List.of(), end);
    }


    public Command makeBasicAutoWithTurn(Pose2d start, List<Translation2d> midpoints,Pose2d end, double gyroAngleDegrees){
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
        .andThen(()->rc.chassis.driveToBearing(Math.toRadians(gyroAngleDegrees)))
        // .andThen(()->rc.chassis.resetOdometry(new Pose2d(start.getX(),start.getY(), rc.navx.getRotation2d())))
        .andThen(rc.sequenceFactory.getSetRPMandShootCommand(5500,50))
        .andThen(new ParallelDeadlineGroup(
            generateSwerveControllerCommand(start, midpoints, end).andThen(new WaitCommand(1)).withTimeout(6),
            rc.sequenceFactory.getIntakeThenAlignCommand()
        ))
        ;
    }

    public Command makeNoteToMidIntake(Pose2d start, List<Translation2d> midpoints, Pose2d end){
        return new InstantCommand()
        .andThen(new ParallelDeadlineGroup(
            generateSwerveControllerCommand(start, midpoints, end).andThen(new WaitCommand(1)).withTimeout(6),
            rc.sequenceFactory.getIntakeThenAlignCommand()
        ))
        .andThen(generateSwerveControllerCommand(end, midpoints, start).withTimeout(6))
        ;
    }

    public Command makeNoteToMidIntake(Pose2d start, Pose2d end){
        return makeNoteToMidIntake(start, List.of(), end);
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
