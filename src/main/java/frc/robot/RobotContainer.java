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
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ChassisConstants.DriveConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ClimberGoHome;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.PassthroughAlignNote;
import frc.robot.commands.SetDunkArmProfiled;
import frc.robot.commands.SetDunkArmSlew;
import frc.robot.commands.SetShooterProfiled;
import frc.robot.commands.VisionTurnToAprilTag;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DunkArm;
import frc.robot.subsystems.DunkArmRoller;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeVision;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Passthrough;
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
  public PowerDistribution pdh = new PowerDistribution(30, ModuleType.kRev);
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
  public final LEDs leds = new LEDs();
  public final DunkArm dunkArm = new DunkArm();
  public final DunkArmRoller dunkArmRoller = new DunkArmRoller();
  //TODO: Vision Needs access to pose estimator: Either by objects in 
  // Robotcontainer or via a method in Chassis
  
  //Keep Sequences and Autos in a single place 
  public final SequenceFactory sequenceFactory;
  public final AutoFactory autoFactory;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final CommandXboxController driverController = new CommandXboxController(0);
  public final CommandJoystick operatorJoystick = new CommandJoystick(1);

  public final IntakeVision intakeVision = new IntakeVision(navx, swerveDrivePoseEstimator);
  public final ShooterVision shooterVision = new ShooterVision(navx, swerveDrivePoseEstimator);

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
  }

  private void configureDefaultCommands() {
    // The left stick controls translation of the robot.
    // Turning is controlled by the X axis of the right stick.
  //TODO: This should work now
  chassis.setDefaultCommand(chassis.getFCDriveCommand( 
    ()-> -driverController.getLeftY(), 
    ()-> -driverController.getLeftX(), 
    ()-> -driverController.getRightX()
  ));

    //default, but only runs once
    //TODO: Only enable when robot is tested 
    new Trigger(DriverStation::isEnabled)
    .and(()->climber.isHomed==false)
    .whileTrue(new ClimberGoHome(climber));
    
    leds.setDefaultCommand(leds.showTeamColor());
    new Trigger(passthrough::isBlocked).onTrue(leds.showNoteIntake());
    //TODO: When shooter is aligned with target, and at rpm, show show green lights


    shooterFlywheel.setDefaultCommand(shooterFlywheel.getShooterSetRPMCommand(0));
    
    //align a note if nothing else is using passthrough
    new Trigger(DriverStation::isEnabled)
    .and(passthrough::isBlocked)
    .and(()->passthrough.getCurrentCommand()==null)
    .whileTrue(new PassthroughAlignNote(passthrough, intake))
    ;

    intake.setDefaultCommand(new RunCommand(()->{intake.setPower(0.0);}, intake));

    dunkArm.setDefaultCommand(new SetDunkArmSlew(-25, dunkArm)
      .andThen(new WaitCommand(1))
      .andThen(new RunCommand(()->dunkArm.setPower(0), dunkArm))
    );
    dunkArmRoller.setDefaultCommand(new RunCommand(()->{dunkArmRoller.stop();}, dunkArmRoller));
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
    driverController.button(5).whileTrue(chassis.getFCDriveCommand(()->-driverController.getLeftY()/2.0, ()->driverController.getLeftX()/2.0, ()->driverController.getRightX()/2.0));

    driverController.button(6).whileTrue(
      new VisionTurnToAprilTag(shooterVision, intakeVision, chassis)
    );

    driverController.button(7).onTrue(new ClimberGoHome(climber));

    //Reset Gyro
    driverController.button(8).onTrue(new InstantCommand()
    .andThen(new InstantCommand(()-> chassis.zeroHeading(), chassis)));

    driverController
    .axisGreaterThan(3, 0.5)
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

    operatorJoystick.button(1)
    .onTrue(new ConditionalCommand(
      new RunCommand(passthrough::intake,passthrough)
        .alongWith(new RunCommand(intake::intake,intake))
        .withTimeout(5), 
      new RunCommand(()->dunkArmRoller.setSpeed(-0.4), dunkArmRoller)
        .withTimeout(3),//dunk arm roller for amp
      passthrough::isBlocked
    ));

    operatorJoystick.button(2).onTrue(new ParallelCommandGroup(
      new SetShooterProfiled(0, shooter), //TODO: not setting to 0
      //.andThen(()->shooter.setAngle(0.0)) //TODO: not setting to 0
      new SetDunkArmSlew(-25, dunkArm).runForever()
      ));

    //empty button?
    operatorJoystick.button(3)
    // .whileTrue(shooterFlywheel.getShooterSetRPMCommand(2500));
    .whileTrue(new ParallelCommandGroup(
      shooterFlywheel.getShooterSetRPMCommand(10000),
      new SetShooterProfiled(50, shooter)
    ));

    operatorJoystick.button(4).onTrue(
      new SetShooterProfiled(50, shooter)
      //TODO: vision targeting to speaker angle
    );

    operatorJoystick.button(5).whileTrue(
      //Trying out sequence factory
      sequenceFactory.getDunkArmNoteTransferSequence()
    );

    operatorJoystick.button(6).whileTrue(
      new SetDunkArmSlew(105, dunkArm).runForever()
    );

    operatorJoystick.button(7).whileTrue(new ParallelCommandGroup(
      new RunCommand(intake::eject, intake),
      new RunCommand(passthrough::eject, passthrough),
      new SetShooterProfiled(0, shooter)
    )//TODO: set shooter/intake eject RPM properly
    .finallyDo((e)->passthrough.stop())
    )
    ;

    operatorJoystick.button(8).whileTrue(new RunCommand(//TODO: check if wrok 
      ()->climber.setPosition(climber.kMaxHeight),
      climber)
    );

    operatorJoystick.button(9).whileTrue(new RunCommand(//TODO: check if wrok 
      ()->climber.setPosition(Units.Inches.of(9.5)),
      climber)
    );

    operatorJoystick.button(10).whileTrue(
      // new IntakeNote(intake, passthrough)
      new ParallelCommandGroup(
        new IntakeNote(intake, passthrough),
        new SetShooterProfiled(0, shooter)
      )
    );

    operatorJoystick.button(11).whileTrue(
      new RunCommand(()->dunkArm.setPower(operatorJoystick.getRawAxis(1)), dunkArm)
    );
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }

  //NOTE: There's a built in function for this! Let's use that.
  // BooleanSupplier inDeadzone = ()->{return (Math.sqrt(Math.pow(driverController.getRawAxis(2), 2) + Math.pow(driverController.getRawAxis(3), 2))) > 0.5;};
  private boolean isRightStickInDeadzone(){
    return  Math.hypot(driverController.getRawAxis(4), driverController.getRawAxis(5)) > 0.5;
  }

}
