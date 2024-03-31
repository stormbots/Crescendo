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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
import frc.robot.commands.VisionTrackNote;
import frc.robot.commands.VisionTurnToSpeakerOpticalOnly;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DunkArm;
import frc.robot.subsystems.DunkArmRoller;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeVision;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Passthrough;
import frc.robot.subsystems.PowerManager;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.ShooterVision;

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

  public static final double INITIALSHOOTEROFFSET =1.0; 
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

    SmartDashboard.putNumber("navx/angle", navx.getRotation2d().getDegrees());
    // SmartDashboard.putData("shooter/profile0", new SetShooterProfiled(0.0, shooter));
    // SmartDashboard.putData("shooter/profile30", new SetShooterProfiled(30, shooter));
    // SmartDashboard.putData("shooter/profile45", new SetShooterProfiled(45.0, shooter));
    // SmartDashboard.putData("shooter/profile60", new SetShooterProfiled(60.0, shooter));
    // SmartDashboard.putData("shooter/pidset0", shooter.getDebugSetAngle(0.0));
    // SmartDashboard.putData("shooter/pidset30", shooter.getDebugSetAngle(30.0));
    // SmartDashboard.putData("shooter/pidset60", shooter.getDebugSetAngle(60.0));

    // SmartDashboard.putData("shooterFlywheel/set0",shooterFlywheel.getShooterSetRPMCommand(0));
    // SmartDashboard.putData("shooterFlywheel/set1000",shooterFlywheel.getShooterSetRPMCommand(1000));
    // SmartDashboard.putData("shooterFlywheel/set2500",shooterFlywheel.getShooterSetRPMCommand(2500));
    // SmartDashboard.putData("shooterFlywheel/set5000",shooterFlywheel.getShooterSetRPMCommand(5000));

    // SmartDashboard.putData("dunkArm/setProfile-20", new SetDunkArmProfiled(-20, dunkArm));
    // SmartDashboard.putData("dunkArm/setProfile0", new SetDunkArmProfiled(0, dunkArm));
    // SmartDashboard.putData("dunkArm/setProfile20", new SetDunkArmProfiled(20, dunkArm));
    // SmartDashboard.putData("dunkArm/setProfile85", new SetDunkArmProfiled(85, dunkArm));
    
    // SmartDashboard.putData("dunkArm/setPID-20", new RunCommand(()->dunkArm.setArmAngle(-20), dunkArm));
    // SmartDashboard.putData("dunkArm/setPID0", new RunCommand(()->dunkArm.setArmAngle(0), dunkArm));
    // SmartDashboard.putData("dunkArm/setPID20", new RunCommand(()->dunkArm.setArmAngle(20), dunkArm));
    // SmartDashboard.putData("dunkArm/setPID85", new RunCommand(()->dunkArm.setArmAngle(85), dunkArm));

    // SmartDashboard.putData("dunkArm/setSlew-20", new SetDunkArmSlew(-20, dunkArm));
    // SmartDashboard.putData("dunkArm/setSlew0", new SetDunkArmSlew(0, dunkArm));
    // SmartDashboard.putData("dunkArm/setSlew20", new SetDunkArmSlew(20, dunkArm));
    // SmartDashboard.putData("dunkArm/setSlew85", new SetDunkArmSlew(85, dunkArm));
    // SmartDashboard.putData("NoteTransferToDunkArm/tester", sequenceFactory.getDunkArmNoteTransferSequence());
    // SmartDashboard.putData("IntakeNote", new IntakeNote(intake, passthrough));
    // SmartDashboard.putData("RunRollers", new RunCommand(()-> dunkArmRoller.intake(), dunkArmRoller));
    SmartDashboard.putData("rumble", new DriverFeedback(driverController, ()->true));

    SmartDashboard.putData("restTo0,0", new InstantCommand(()->chassis.resetOdometry(new Pose2d())));
    SmartDashboard.putData("restToSpeaker", new InstantCommand(()->chassis.resetOdometry(new Pose2d(new Translation2d(1.2, 5.5), new Rotation2d()))));

    SmartDashboard.putData("intakevision/tracknotecommand", new VisionTrackNote(
      ()-> -driverController.getLeftX(), ()-> -driverController.getLeftY(), ()-> -driverTurnJoystickValue(),
      chassis,  intake, passthrough, intakeVision,  leds
    ));
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

    new Trigger(DriverStation::isEnabled)
    .and(()->shooter.isHomed==false)
    .whileTrue(new CalibrateShooter(shooter));

    new Trigger( ()-> dunkArm.getAngle()>45 )
    .onTrue(new InstantCommand(()->climber.setReverseSoftLimit(0.1)))
    .onFalse(new InstantCommand(()->climber.setReverseSoftLimit(9.5)))
    ;

    new Trigger(DriverStation::isTeleop)
    .and(()-> Timer.getMatchTime()<20)
    .onTrue(
      new RunCommand(()->leds.setColor(Color.kPurple),leds).withTimeout(2)
    );
    
    leds.setDefaultCommand(leds.set5vLedStrip().andThen(leds.showTeamColor()));

    new Trigger(intake::isBlocked)
    .onTrue(leds.showNoteIntake())
    .onTrue(
      new DriverFeedback(driverController, intake::isBlocked)
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

    var teleopdunkarm = new SetDunkArmSlew(-25, dunkArm)
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
      //new StartEndCommand(dunkArm, onEnd, requirements)
      // new RunCommand(()->dunkArmRoller.setSpeed(0), dunkArmRoller)
      // ); 

    // shooterVision.setDefaultCommand(new StartEndCommand(()->shooterVision.setPipeline(ShooterVision.LimelightPipeline.kNoZoom), ()->{}, shooterVision));
  }

  private void configureDriverBindings() {
    // // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // // cancelling on release.
    // driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    // driverController.x().whileTrue(
    //   new VisionTurnToAprilTag(shooterVision, intakeVision, chassis)
    // );

    //face toward driver
    driverController.button(1).whileTrue(chassis.getDriveToBearingCommand(()-> -driverController.getLeftY(), ()-> -driverController.getLeftX(), ()->Units.Degrees.of(180))); //Face toward driver
    driverController.button(2).whileTrue(chassis.getDriveToBearingCommand(()-> -driverController.getLeftY(), ()-> -driverController.getLeftX(), ()->Units.Degrees.of(270))); //Face right
    driverController.button(3).whileTrue(chassis.getDriveToBearingCommand(()-> -driverController.getLeftY(), ()-> -driverController.getLeftX(), ()->Units.Degrees.of(90))); //Face left
    driverController.button(4).whileTrue(chassis.getDriveToBearingCommand(()-> -driverController.getLeftY(), ()-> -driverController.getLeftX(), ()->Units.Degrees.of(0))); //Face away from driver
    driverController.button(5).whileTrue(chassis.getFCDriveCommand(()->-driverController.getLeftY()/2.0, ()->-driverController.getLeftX()/2.0, ()->-driverTurnJoystickValue()/2.0));

    driverController.button(6).whileTrue(
      new VisionTurnToSpeakerOpticalOnly(
        ()-> -driverController.getLeftY(),
        ()-> -driverController.getLeftX(),
        ()-> -driverTurnJoystickValue(),
        shooterVision, chassis, navx)
      )
      .whileTrue(
        new ShooterSetVision(shooter, shooterVision, shooterFlywheel).runForever()
      )
      .whileTrue(leds.readyLights(shooterFlywheel::isOnTarget, shooter::isOnTarget))
      .onTrue(
        new DriverFeedback(driverController, shooterFlywheel::isOnTarget, shooter::isOnTarget)
        .withTimeout(2)
      )
    ;

    driverController.button(7).onTrue(new ClimberGoHome(climber));

    //Reset Gyro
    driverController.button(8).onTrue(new InstantCommand()
    .andThen(new InstantCommand(()-> chassis.setFieldCentricOffset(0.0), chassis)));

    driverController
    .axisGreaterThan(2, 0.5)
    // .and(()->isRightStickInDeadzone()==false)
    .whileTrue(
      chassis.getDriveToBearingCommand(
        ()-> -driverController.getLeftY(), 
        ()-> -driverController.getLeftX(), 
        ()->Units.Radians.of(Math.atan2(driverController.getRightY(), -driverController.getRightX())+Math.PI/2))
    // new RunCommand(
    //   () -> chassis.driveToBearing(
    //       -MathUtil.applyDeadband(driverController.getRawAxis(1), OIConstants.kDriveDeadband),
    //       -MathUtil.applyDeadband(driverController.getRawAxis(0), OIConstants.kDriveDeadband),
    //       Math.atan2(-driverController.getRightY(), driverController.getRightX())
    //       ),
    //   chassis)
    );
   
   


    // Limelight Intake Vision
    driverController
    .axisGreaterThan(3, 0.5) //left trigger?
    .whileTrue(
      new VisionTrackNote(
      ()-> -driverController.getLeftY(), 
      ()-> -driverController.getLeftX(),
      ()-> -driverTurnJoystickValue(), 
      chassis, intake, passthrough, intakeVision, leds)
    );

  }


  private void configureOperatorBindings(){
    // operatorJoystick.button(2).onTrue(new InstantCommand()
    //   .andThen( new SetDunkArmSlew(0, dunkArm))
    // );
    // // operatorJoystick.button(1).whileTrue(new InstantCommand());
    // operatorJoystick.button(9)
    // .whileTrue(new IntakeNote(intake, passthrough));

    // operatorJoystick.button(10)
    // .whileTrue(new RunCommand(intake::eject, intake));

    // operatorJoystick.button(3).onTrue(new InstantCommand()
    //   .andThen(new SetDunkArmSlew(100, dunkArm))
    // );
    // operatorJoystick.button(1).whileTrue(new InstantCommand());

    //old score button, keeping just because 
    // operatorJoystick.button(1)
    // .whileTrue(new ConditionalCommand(
    //   //Shoot using whatever shooter position/speed is set up elsewhere
    //   new RunCommand(passthrough::intake,passthrough).finallyDo(passthrough::stop)
    //     .alongWith(new RunCommand(intake::intake,intake).finallyDo(intake::stop))
    //     .withTimeout(5),
    //   //score out of rollers
    //   new RunCommand(dunkArmRoller::scoreTrap, dunkArmRoller)
    //     .withTimeout(3),
    //   passthrough::isBlocked
    // ))
    // .onTrue(
    //   new LogOutgoingShot(swerveDrivePoseEstimator, shooter, shooterFlywheel)
    // );
    
    //Score button
    operatorJoystick.button(1)
    .and(()->passthrough.isBlocked()||intake.isBlocked())
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
    .and(()->!passthrough.isBlocked()&&!intake.isBlocked())
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
    operatorJoystick.button(2).onTrue(new ParallelCommandGroup(
      new SetShooterProfiled(0, shooter), //TODO: not setting to 0
      new SetDunkArmSlew(-25, dunkArm)
      ).withTimeout(3)
      )
      ;

    //speaker shot
    operatorJoystick.button(3)
    .whileTrue(new ParallelCommandGroup(
      // new SetFlywheelSlew(4000, shooterFlywheel),
      new RunCommand(()->shooterFlywheel.setRPM(4000),shooterFlywheel),
      new SetShooterProfiled(40.5+0.5, shooter).runForever())
    )
    .whileTrue(leds.readyLights(shooterFlywheel::isOnTarget, shooter::isOnTarget));

    //podium/far shot
    operatorJoystick.button(4) //far shooting
    .whileTrue(new ParallelCommandGroup(
      new SetFlywheelSlew(4500, shooterFlywheel),
      new SetShooterProfiled(20+1, shooter).runForever())
    )
    .whileTrue(leds.readyLights(shooterFlywheel::isOnTarget, shooter::isOnTarget)
    )
    ;

    //load rollers / intake to rollers
    operatorJoystick.button(5).whileTrue(
      new ConditionalCommand(
        sequenceFactory.getDunkArmNoteTransferSequence(),

        new ParallelCommandGroup(
          new IntakeNote(intake, passthrough).andThen(new PassthroughAlignNote(passthrough, intake)),
          new SetShooterProfiled(0, shooter),
          //Unnecesary change, made to warm up flywheel while debugging, may still be wanted
          new SetFlywheelSlew(500, shooterFlywheel)
        )
        .until(passthrough::isBlocked)
        .andThen(sequenceFactory.getDunkArmNoteTransferSequence())
        .andThen(new SetDunkArmSlew(-25, dunkArm)),
        
        passthrough::isBlocked
      )
    )
    ;

    //arm to amp
    //TODO: This should be whileHeld, need to validate control issues with operator 
    operatorJoystick.button(6).whileTrue(
      new SetDunkArmSlew(105, dunkArm).runForever()
    );

    //eject note
    operatorJoystick.button(7).whileTrue(new ParallelCommandGroup(
      new RunCommand(intake::eject, intake),
      new RunCommand(passthrough::eject, passthrough),
      new SetShooterProfiled(0, shooter), 
      new SetFlywheelSlew(-3000, shooterFlywheel),
      new RunCommand(dunkArmRoller::eject, dunkArmRoller)
    )//TODO: set shooter/intake eject RPM properly
    .finallyDo((e)->passthrough.stop())
    )
    ;

    //intake note
    operatorJoystick.button(8).whileTrue(
      new IntakeNote(intake, passthrough)
      .andThen(new PassthroughAlignNote(passthrough,intake))
    )
    // .whileTrue(new SetFlywheelSlew(0, shooterFlywheel))
    .whileTrue(new SetShooterProfiled(0, shooter))
    ;

    //DEBUG CODE: manually adjust rollers
    // operatorJoystick.button(9).whileTrue(
    //   new RunCommand(()->dunkArmRoller.setSpeed(operatorJoystick.getRawAxis(3)*-60*0.05*0.05*0.5), dunkArmRoller)
    // );0

    //move dunkarm manually
    operatorJoystick.button(10).onTrue(
      new RunCommand(()->dunkArm.setPowerFF(-.25*operatorJoystick.getRawAxis(1)), dunkArm)
    )
    .whileTrue( 
      new DunkArmRollerHoldNote(dunkArm, dunkArmRoller)
    )
    .onTrue(new ConditionalCommand(
      new ClimberSetPosition(climber, Units.Inches.of(11)),
      new InstantCommand(), 
      ()->climber.getPosition().in(Units.Inches)<2.0
    ))
    ;

    operatorJoystick.button(11).onTrue(
      new InstantCommand(()->dunkArm.syncEncoders())
    );

    //climbers up
    operatorJoystick.button(13).whileTrue(
      new ClimberSetPosition(climber, climber.kMaxHeight)
    );

    //climbers down
    operatorJoystick.button(14).whileTrue(
      new ClimberSetPosition(climber, Units.Inches.of(1.0))
    );

    // operatorJoystick.button(15).whileTrue(
    //   new RunCommand(()->shooterFlywheel.setRPMProfiled(4000), shooterFlywheel)
    // );

    operatorJoystick.button(16).whileTrue(
      new ShooterSetVision(shooter, shooterVision, shooterFlywheel).runForever()
    )
    .whileTrue(leds.readyLights(shooterFlywheel::isOnTarget, shooter::isOnTarget)
    );

    // Used for testing only.

    // operatorJoystick.button(12)
    // .whileTrue(
    //   new ShooterSetManually(shooter, shooterFlywheel, ()->operatorJoystick.getRawAxis(3))
    // )
    // .whileTrue(leds.readyLights(shooterFlywheel::isOnTarget, shooter::isOnTarget));

    // operatorJoystick.button(12).whileTrue(new ParallelCommandGroup( //shooting across field, not tested
    //   new SetDunkArmSlew(20, dunkArm),
    //   new SetShooterProfiled(15, shooter),
    //   new SetFlywheelSlew(4000, shooterFlywheel)
    // ));
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);

    return autoFactory.getAutoChooser().getSelected();
    // return autoFactory.getBlueBottomAuto();

  }

  private double driverTurnJoystickValue(){
    var stick = driverController.getRightX();
    stick = stick*stick*Math.signum(stick); 
    // stick /= 40;
    return stick;
  }
}
