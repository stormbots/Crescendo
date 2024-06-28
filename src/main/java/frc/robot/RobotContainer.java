// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ChassisConstants.DriveConstants;
import frc.robot.commands.CalibrateShooter;
import frc.robot.commands.ClimberGoHome;
import frc.robot.commands.ClimberSetPosition;
import frc.robot.commands.DriverFeedback;
import frc.robot.commands.DunkArmRollerHoldNote;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.LogOutgoingShot;
import frc.robot.commands.PassthroughAlignNote;
import frc.robot.commands.SetDunkArmSlew;
import frc.robot.commands.SetFlywheelSlew;
import frc.robot.commands.SetShooterProfiled;
import frc.robot.commands.ShooterSetManually;
import frc.robot.commands.ShooterSetVision;
import frc.robot.commands.ShooterSetVisionLob;
import frc.robot.commands.VisionTrackNote;
import frc.robot.commands.VisionTrackNoteAuto;
import frc.robot.commands.VisionTurnToSpeakerOpticalOnly;
import frc.robot.commands.VisionTurnToTargetPose;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DunkArm;
import frc.robot.subsystems.DunkArmRoller;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeVision;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Passthrough;
import frc.robot.subsystems.PassthroughLock;
import frc.robot.subsystems.PowerManager;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.ShooterVision;
import frc.robot.subsystems.ShooterVision.LimelightPipeline;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // public PowerDistribution pdh = new PowerDistribution(30, ModuleType.kRev);

  public static final double INITIALSHOOTEROFFSET = 0; 
  public static double shooterOffset = INITIALSHOOTEROFFSET;//adjusted via slider
  public SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
    new Translation2d(DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2),
    new Translation2d(DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2),
    new Translation2d(-DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2),
    new Translation2d(-DriveConstants.kWheelBase / 2, -DriveConstants.kTrackWidth / 2)
  );
  public AHRS navx = new AHRS();
  public SwerveDrivePoseEstimator swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(swerveDriveKinematics, navx.getRotation2d(), 
    new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    }, 
    new Pose2d(0, 0 , new Rotation2d())
  );

  public Field2d field = new Field2d();
  public final Chassis chassis = new Chassis(navx, swerveDriveKinematics, swerveDrivePoseEstimator, field);
  public final Climber climber = new Climber(navx);
  public final Intake intake = new Intake();
  public final Passthrough passthrough = new Passthrough();
  public final Shooter shooter = new Shooter();
  public final ShooterFlywheel shooterFlywheel = new ShooterFlywheel();
  public final Leds leds = new Leds();
  public final DunkArm dunkArm = new DunkArm();
  public final DunkArmRoller dunkArmRoller = new DunkArmRoller();
  //TODO: Vision Needs access to pose estimator: Either by objects in 
  // Robotcontainer or via a method in Chassis

  public final IntakeVision intakeVision = new IntakeVision();
  // public final IntakeVision intakeVision = new IntakeVision();
  public final ShooterVision shooterVision = new ShooterVision(swerveDrivePoseEstimator);
  

  //Keep Sequences and Autos in a single place 
  public final SequenceFactory sequenceFactory;
  public final AutoFactory autoFactory;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final CommandXboxController driverController = new CommandXboxController(0);
  public final CommandJoystick operatorJoystick = new CommandJoystick(1);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Run delayed constructors
    sequenceFactory = new SequenceFactory(this);
    autoFactory = new AutoFactory(this);

    // Sensor Driven triggers/commands
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Configure the trigger bindings
    configureDefaultCommands();
    configureDriverBindings();
    configureOperatorBindings();

    var pm = PowerManager.getInstance()
    .setRobotPowerBudget(280)
    // .addChassisSystem(chassis, 200, 260, chassis::setCurrentLimits)
    .addSystem(dunkArm, 40)
    .addSystem(intake, 25)
    .addSystem(passthrough, 20)
    .addSystem(shooter, 0)
    .addSystem(shooterFlywheel, 80)
    ;
  }

  private void configureDefaultCommands() {
    // The left stick controls translation of the robot.
    // Turning is controlled by the X axis of the right stick.
  //TODO: This should work now
    chassis.setDefaultCommand(chassis.getFCDriveCommand( 
      ()-> -driverController.getLeftY(), 
      ()-> -driverController.getLeftX(), 
      ()-> -driverTurnJoystickValue()// divide by 4 is still not enough
    ));

    //default, but only runs once
    //TODO: Only enable when robot is tested 
    new Trigger(DriverStation::isEnabled)
    .and(()->climber.isHomed==false)
    .whileTrue(new ClimberGoHome(climber).withTimeout(15));

    // new Trigger(DriverStation::isEnabled)
    // .and(()->shooter.isHomed==false)
    // .whileTrue(new CalibrateShooter(shooter));

    new Trigger( ()-> dunkArm.getAngle()>45 )
    .onTrue(new InstantCommand(()->climber.setReverseSoftLimit(climber.climbingReverseSoftLimit)).ignoringDisable(true))
    .onFalse(
      new InstantCommand(()->climber.setReverseSoftLimit(climber.defaultReverseSoftLimit)).ignoringDisable(true)
    )
    ;
    
    leds.setDefaultCommand
      (leds.set5vLedStrip().andThen(leds.showTeamColor())
    );

    new Trigger(intake::isBlocked)
    .onTrue(leds.showNoteIntake())
    .onTrue(
      new DriverFeedback(shooterVision, driverController, intake::isBlocked)
      .withTimeout(2)
    );
    //TODO: When shooter is aligned with target, and at rpm, show show green lights

    shooterFlywheel.setDefaultCommand(
      new WaitCommand(0.5)
      // .andThen(
      //   new RunCommand(()->shooterFlywheel.setRPM(0), shooterFlywheel)
      //   .until(shooterFlywheel::isOnTarget)
      //   )
      .andThen(new RunCommand(shooterFlywheel::stop,shooterFlywheel))
    );
    
    //align a note if nothing else is using passthrough
    new Trigger(DriverStation::isTeleop)
    .and(()->(passthrough.isBlocked() || intake.isBlocked()))
    .and(()->passthrough.getCurrentCommand()==null)
    .whileTrue(new PassthroughAlignNote(passthrough, intake))
    ;

    intake.setDefaultCommand(new RunCommand(()->{intake.stop();}, intake));

    shooter.setDefaultCommand( //TODO: why jank?? :(
      new WaitCommand(1)
      .andThen(new SetShooterProfiled(0, shooter)
        .withTimeout(1)
      )
      .andThen(new WaitCommand(0.1))
      .andThen(new RunCommand(shooter::stopShooter))
    );

    var teleopdunkarm = new SetDunkArmSlew(-30, dunkArm) //might as well
    .andThen(new WaitCommand(0.2))
    .andThen(new RunCommand(()->dunkArm.stop(), dunkArm));
    
    dunkArm.setDefaultCommand(new ConditionalCommand(
      teleopdunkarm,
      new RunCommand(()->{}, dunkArm),
      DriverStation::isTeleop
      )
    );

    //TODO: Have dunkarm hold note.
    dunkArmRoller.setDefaultCommand(new StartEndCommand(
      ()->{dunkArmRoller.setIdleMode(IdleMode.kCoast); dunkArmRoller.stop();},
      ()->dunkArmRoller.setIdleMode(IdleMode.kBrake), 
      dunkArmRoller
    ));
    // shooterVision.setDefaultCommand(new StartEndCommand(()->shooterVision.setPipeline(ShooterVision.LimelightPipeline.kNoZoom), ()->{}, shooterVision));
  }

  private void configureDriverBindings() {

    //face toward driver
    driverController.button(1).whileTrue(chassis.getDriveToBearingCommand(()-> -driverController.getLeftY(), ()-> -driverController.getLeftX(), ()->Units.Degrees.of(180))); //Face toward driver
    driverController.button(2).whileTrue(chassis.getDriveToBearingCommand(()-> -driverController.getLeftY(), ()-> -driverController.getLeftX(), ()->Units.Degrees.of(270))); //Face right
    driverController.button(3).whileTrue(chassis.getDriveToBearingCommand(()-> -driverController.getLeftY(), ()-> -driverController.getLeftX(), ()->Units.Degrees.of(90))); //Face left
    driverController.button(4).whileTrue(chassis.getDriveToBearingCommand(()-> -driverController.getLeftY(), ()-> -driverController.getLeftX(), ()->Units.Degrees.of(0))); //Face away from driver
   // driverController.button(5).whileTrue(chassis.getFCDriveCommand(()->-driverController.getLeftY()/5.0, ()->-driverController.getLeftX()/5.0, ()->-driverTurnJoystickValue()/5.0));

    driverController.button(5)
    .and(shooterVision::distanceInRange)
    .debounce(0.1)
    .whileTrue(new VisionTurnToSpeakerOpticalOnly(
        ()-> -driverController.getLeftY()/5.0*.4,
        ()-> -driverController.getLeftX()/5.0*0.5,
        ()-> -driverTurnJoystickValue()/5.0,
        shooterVision, chassis, navx)
    )
    ;

    driverController.button(5)
    .or(driverController.button(6))
    .whileTrue(
      new VisionTurnToSpeakerOpticalOnly(
        ()-> -driverController.getLeftY(),
        ()-> -driverController.getLeftX(),
        ()-> -driverTurnJoystickValue(),
        shooterVision, chassis, navx)
    )
    .whileTrue(
      new ShooterSetVision(shooter, shooterVision, shooterFlywheel).runForever()
    )
    .whileTrue(leds.readyLightsPossible(shooterVision::distanceInRange, shooterFlywheel::isOnTarget,shooter::isOnTarget)
    )
    .onTrue(
      new DriverFeedback(driverController, shooterFlywheel::isOnTarget, shooter::isOnTarget)
      .withTimeout(2)
    )
    .onTrue(
      PassthroughLock.setUnlocked()
    )
    ;

    driverController.button(7).onTrue(new ClimberGoHome(climber)
    .andThen(new InstantCommand(()->shooter.stopShooter()))
    .andThen(new WaitCommand(0.1))
    .alongWith(new CalibrateShooter(shooter)))
    .onTrue(
      new RunCommand(()->dunkArm.stop(), dunkArm).withTimeout(0.2)
      .andThen(new InstantCommand(()->dunkArm.syncEncoders()))
    );

    //Reset Gyro
    driverController.button(8).onTrue(new InstantCommand()
    .andThen(new InstantCommand(()-> chassis.setFieldCentricOffset(0.0), chassis))
    );

    driverController
    .axisGreaterThan(2, 0.5)
    // .and(()->isRightStickInDeadzone()==false)
    .whileTrue(
      chassis.getDriveToBearingCommand(
        ()-> -driverController.getLeftY(), 
        ()-> -driverController.getLeftX(), 
        ()->Units.Radians.of(Math.atan2(driverController.getRightY(), -driverController.getRightX())+Math.PI/2))
    );

   driverController.povDown()
  //  .whileTrue(
  //     new ShooterSetVisionLob(shooter, shooterVision, shooterFlywheel).runForever()
  //   )
    .whileTrue(
      new VisionTurnToTargetPose(
        ()-> -driverController.getLeftY(),
        ()-> -driverController.getLeftX(),
        ()-> -driverTurnJoystickValue(), shooterVision, chassis, navx, swerveDrivePoseEstimator, shooterVision.getField())
        .reverseDirection() 
    )
    .onTrue(PassthroughLock.setUnlocked())
    .whileTrue(leds.readyLights(shooter::isOnTarget, shooterFlywheel::isOnTarget))
    .whileTrue(new StartEndCommand(shooterVision::selectAllTagsPipeline, shooterVision::selectSpeakerPipeline)
    )
    ;


    // // Limelight Intake Vision
    driverController
    .axisGreaterThan(3, 0.5) //left trigger?
    .onTrue(PassthroughLock.setLocked())
    .whileTrue(
      new SetShooterProfiled(0, shooter)
    )
    .whileTrue(
      new RunCommand(()->{}).until(shooter::isReadyToIntake)
      .andThen(
        new VisionTrackNote(
        ()-> -driverController.getLeftY(), 
        ()-> -driverController.getLeftX(),
        ()-> -driverTurnJoystickValue(), 
        chassis, intake, passthrough, intakeVision, leds)
      )
    )
    ;

    driverController.povRight()
    .whileTrue(new VisionTrackNoteAuto(()->0.5, ()->0.0, ()->0.0, chassis, intake, passthrough, intakeVision, leds));


    driverController.povUp()
    .whileTrue(
      new RunCommand(()->chassis.setX())
    );

    // driverController.povLeft()
    // .whileTrue(
    //   new InstantCommand(()->chassis.resetOdometryAllianceManaged(new Pose2d(2, 5, new Rotation2d())))
    //   .andThen(autoFactory.makePathFindToPoseCommand(new Pose2d(2.8, 5.6, new Rotation2d())))
    // );

  }


  private void configureOperatorBindings(){

    operatorJoystick.povCenter().negate()
      .whileTrue(new ParallelCommandGroup(
        new SetDunkArmSlew(0, dunkArm).runForever(),
        new SetShooterProfiled(0, shooter),
        new SetFlywheelSlew(3550, shooterFlywheel)
      )
    )
    .onTrue(PassthroughLock.setUnlocked());
    
    Trigger isSensorBlocked = new Trigger(()->passthrough.isBlocked()||intake.isBlocked());
    Trigger readyToFire = new Trigger(()->
      shooterVision.hasValidTarget() && //UNLESS there is some latency disparity (slightly unsafe but we'll see), this will prevent any empty optional from breaking code as it will never run
   //  Math.abs(shooterVision.getVisibleTargetData().get().angleHorizontal-navx.getRotation2d().getDegrees()) < 10 && //need to tune / not needed? seemed to work fine without it
      shooter.isOnTarget() && 
      shooterFlywheel.isOnTarget() && 
      shooterVision.distanceInRange() //&& 
    //  navx.getRate()<50 //need to tune / not needed? seemed to work fine without it
    )
    .debounce(0.075) //tuned debounce from 0.1 to 0.075
    ;

    driverController.button(5)
    .and(readyToFire.negate())
    .debounce(0.1)//If we never pass readyToFire check, pressing button 1 will do nothing. with debounce, pressing but 1 with but 5 will force a shot (prob better strat)
    .whileTrue(
      new IntakeNote(intake, passthrough) 
      .andThen(new PassthroughAlignNote(passthrough,intake)) //may not work when shooter is up, will push too far INTO spun up wheels
      .withInterruptBehavior(InterruptionBehavior.kCancelIncoming) //I believe we would want to interupt (conflicts with next few lines)
    );

// the order of the trigger above and below matters, dont change it before asking me - Michael H

    driverController.button(5)
    .and(readyToFire)
    .whileTrue(
      new RunCommand(passthrough::intake,passthrough).finallyDo(passthrough::stop)
        .alongWith(new RunCommand(intake::intake,intake).finallyDo(intake::stop))
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming) //I believe we would want to interupt (conflicts with next few lines)
    );

    //Score button
    operatorJoystick.button(1)
    .and(isSensorBlocked)
    .whileTrue(
      //Shoot using whatever shooter position/speed is set up elsewhere
      new RunCommand(passthrough::intake,passthrough).finallyDo(passthrough::stop)
        .alongWith(new RunCommand(intake::intake,intake).finallyDo(intake::stop))
        .withTimeout(5)
    )
    .onTrue(
      new LogOutgoingShot(swerveDrivePoseEstimator, shooter, shooterFlywheel)
    );

    //score trap or amp
    operatorJoystick.button(1)
    .and(isSensorBlocked.negate())
    .and(driverController.button(5).negate())
    .whileTrue(
      //score amp
      new ConditionalCommand(
        new RunCommand(dunkArmRoller::scoreAmp, dunkArmRoller)
        .withTimeout(3),
      //score trap
        new RunCommand(dunkArmRoller::scoreTrap, dunkArmRoller)
        .withTimeout(3), 
        operatorJoystick.button(6)
      )
    );

    //defense position
    operatorJoystick.button(2)
    .onTrue(new ParallelCommandGroup(
      new SetShooterProfiled(0, shooter), //TODO: not setting to 0
      new SetDunkArmSlew(-30, dunkArm)
      ).withTimeout(3)
    )
    ;

    //speaker shot
    operatorJoystick.button(3)
    .whileTrue(new ParallelCommandGroup(
      // new SetFlywheelSlew(4000, shooterFlywheel),
      new RunCommand(()->shooterFlywheel.setRPM(4000),shooterFlywheel),
      new SetShooterProfiled(44.4, shooter).runForever())
    )
    .whileTrue(leds.readyLights(shooterFlywheel::isOnTarget, shooter::isOnTarget))
    .onTrue(
        PassthroughLock.setUnlocked()
    );

    //podium/far shot
    operatorJoystick.button(4) //far shooting
    .whileTrue(new ParallelCommandGroup(
      new ShooterSetVision(shooter, shooterVision, shooterFlywheel).fullRange(),
      new SetDunkArmSlew(0, dunkArm))
    )
    .whileTrue(leds.readyLights(shooterFlywheel::isOnTarget, shooter::isOnTarget))
    .onTrue(
        PassthroughLock.setUnlocked()
    )
    ;

    //load rollers / intake to rollers
    operatorJoystick.button(5).whileTrue(
      new ConditionalCommand(
        sequenceFactory.getDunkArmNoteTransferSequence(),
        // .alongWith(PassthroughLock.setLocked()),

        new ParallelCommandGroup(
          new IntakeNote(intake, passthrough).runForever(),
          new SetShooterProfiled(0, shooter).runForever(),
          new SetFlywheelSlew(ShooterFlywheel.kDunkArmTransferRPM, shooterFlywheel)
        )
        .until(passthrough::isBlocked)
        .andThen(sequenceFactory.getDunkArmNoteTransferSequence())
        .andThen(new SetDunkArmSlew(-30, dunkArm)),
        
        passthrough::isBlocked
      )
    )
    .onTrue(PassthroughLock.setUnlocked())
    ;

    //arm to amp
    operatorJoystick.button(6).whileTrue(
      new SetDunkArmSlew(99.785721, dunkArm).runForever()
    );

    //eject note
    // operatorJoystick.button(7).whileTrue(new ParallelCommandGroup(
    //   new RunCommand(intake::eject, intake),
    //   new RunCommand(passthrough::eject, passthrough),
    //   new SetShooterProfiled(0, shooter), 
    //   new SetFlywheelSlew(-500, shooterFlywheel),
    //   new RunCommand(dunkArmRoller::eject, dunkArmRoller)
    // )//TODO: set shooter/intake eject RPM properly
    // .finallyDo((e)->passthrough.stop())
    // )
    // ;

    operatorJoystick.button(7)
    .whileTrue(new ConditionalCommand(
      new ParallelCommandGroup(
        new RunCommand(intake::eject, intake),
        new RunCommand(passthrough::eject, passthrough),
        new SetShooterProfiled(0, shooter), 
        new SetFlywheelSlew(-500, shooterFlywheel),
        new RunCommand(dunkArmRoller::eject, dunkArmRoller)
      )
      .finallyDo((e)->passthrough.stop()), 

      new ParallelCommandGroup(
        new RunCommand(intake::eject, intake),
        new RunCommand(passthrough::eject, passthrough),
        new SetFlywheelSlew(-500, shooterFlywheel),
        new SetShooterProfiled(0, shooter), 
        new ConditionalCommand(
          new RunCommand(dunkArmRoller::eject, dunkArmRoller),
          new RunCommand(()->{}, dunkArmRoller),
          ()->dunkArm.getAngle()<-20 && shooter.getShooterAngle()<5
        )
      )
      .until(()->passthrough.isBlocked())
      .andThen(new PassthroughAlignNote(passthrough, intake))
      .finallyDo((e)->passthrough.stop()), 

      ()->intake.isBlocked()
    ))
    .onTrue(
        PassthroughLock.setUnlocked()
    );

    //intake note
    operatorJoystick.button(8).whileTrue(
      new RunCommand(()->{}).until(shooter::isReadyToIntake)
      .andThen(new IntakeNote(intake, passthrough))
      .andThen(new PassthroughAlignNote(passthrough,intake))
    )
    .whileTrue(new SetShooterProfiled(0, shooter))
    // .onTrue(new ConditionalCommand(
    //   new ClimberSetPosition(climber, Units.Inches.of(11)),
    //   new InstantCommand(), 
    //   ()->climber.getPosition().in(Units.Inches)<2.0&&climber.isHomed
    //   )
    // )
    .onTrue(PassthroughLock.setLocked()
    )
    .onTrue(
        new DriverFeedback(driverController, shooterFlywheel::isOnTarget, shooter::isOnTarget)
        .withTimeout(2)
    )
    ;

    operatorJoystick.button(9).whileTrue(
      new RunCommand(()->dunkArmRoller.setSpeed(Clamp.clamp(operatorJoystick.getRawAxis(3), 0.1, -0.1)))
    );

    //move dunkarm manually
    operatorJoystick.button(10).onTrue(
      new RunCommand(()->dunkArm.setPowerFF(-.35*operatorJoystick.getRawAxis(1)), dunkArm)
    )
    .onTrue( 
      new DunkArmRollerHoldNote(dunkArm, dunkArmRoller)
    )
    ;

    //climbers up
    operatorJoystick.button(13).whileTrue(
      new ClimberSetPosition(climber, climber.kMaxHeight)
    );

    //climbers down
    operatorJoystick.button(14).whileTrue(
      new ClimberSetPosition(climber, Units.Inches.of(1.0))
    );

    operatorJoystick.button(16).whileTrue(
      new ShooterSetVision(shooter, shooterVision, shooterFlywheel).runForever()
    )
    .whileTrue(leds.readyLights(shooterFlywheel::isOnTarget, shooter::isOnTarget)
    )
    .whileTrue(PassthroughLock.setUnlocked());

    // Used for testing only.

    // operatorJoystick.button(17).whileTrue(
    //   new InstantCommand()
    //   // new RunCommand(()->{
    //   //   chassis.driveToBearing(0, 0, Math.toRadians(Clamp.clamp(60*Math.round((int)navx.getRotation2d().getDegrees()/60), -60, 60)));
    //   // },chassis).until(()->Clamp.bounded(navx.getRotation2d().getDegrees(), Clamp.clamp(60*Math.round((int)navx.getRotation2d().getDegrees()/60), -60, 60)-5, Clamp.clamp(60*Math.round((int)navx.getRotation2d().getDegrees()/60), -60, 60)+5))
    //   .andThen(
    //     new SequentialCommandGroup(new InstantCommand()
    //       .andThen(new SetDunkArmSlew(20, dunkArm))
    //       .andThen(new AutomatedTrap(this))
    //       .andThen(
    //         new ClimberSetPosition(climber, Units.Inches.of(0))
    //         .alongWith(new RunCommand(()->{},dunkArm))
    //       )
    //     )
    //   ).alongWith(new DunkArmRollerHoldNote(dunkArm, dunkArmRoller))
    // )
    // .onFalse(new RunCommand(()->{}, dunkArm))
    // // ;
    operatorJoystick.button(15).whileTrue(
      new ShooterSetVisionLob(shooter, shooterVision, shooterFlywheel).runForever()
    )
    // .whileTrue(
    //   new VisionTurnToTargetPose(
    //     ()-> -driverController.getLeftY(),
    //     ()-> -driverController.getLeftX(),
    //     ()-> -driverTurnJoystickValue(), shooterVision, chassis, navx, swerveDrivePoseEstimator, shooterVision.getField())
    //     .reverseDirection()
      
    // )
    .onTrue(PassthroughLock.setUnlocked())
    .whileTrue(leds.readyLights(shooter::isOnTarget, shooterFlywheel::isOnTarget))
    .whileTrue(new StartEndCommand(shooterVision::selectAllTagsPipeline, shooterVision::selectSpeakerPipeline));

    // operatorJoystick.button(15)
    // .whileTrue(
    //   new ShooterSetManually(shooter, shooterFlywheel, ()->operatorJoystick.getRawAxis(3))
    // )
    // .whileTrue(leds.readyLights(shooterFlywheel::isOnTarget, shooter::isOnTarget));
}

  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    

    return new CalibrateShooter(shooter).andThen(autoFactory.getAutoChooser().getSelected());
    // return autoFactory.getBlueBottomAuto();

  }

  private double driverTurnJoystickValue(){
    var stick = driverController.getRightX();
    stick = stick*stick*Math.signum(stick); 
    // stick /= 40;
    return stick;
  }
}
