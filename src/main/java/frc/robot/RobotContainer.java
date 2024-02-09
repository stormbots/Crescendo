// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.Instant;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.ChassisConstants.DriveConstants;
import frc.robot.ChassisConstants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ClimberGoHome;
import frc.robot.commands.LightingProgressBar;
import frc.robot.commands.SetShooterProfiled;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DunkArm;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeVision;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Passthrough;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterFlywheel;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
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
  public final ShooterFlywheel flywheel = new ShooterFlywheel();
  public final LEDs leds = new LEDs();
  public final DunkArm dunkArm = new DunkArm();
  //TODO: Vision Needs access to pose estimator: Either by objects in 
  // Robotcontainer or via a method in Chassis
  
  //Keep Sequences and Autos in a single place 
  public final SequenceFactory sequenceFactory;
  public final AutoFactory autoFactory;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final CommandXboxController driverController = new CommandXboxController(0);
  public final CommandJoystick operatorJoystick = new CommandJoystick(1);

  public final IntakeVision intakeVision = new IntakeVision(navx, swerveDrivePoseEstimator);
  //public final ShooterVision shooterVision = new ShooterVision(navx, swerveDrivePoseEstimator);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Run delayed constructors
    sequenceFactory = new SequenceFactory(this);
    autoFactory = new AutoFactory(this);


    // Sensor Driven triggers/commands
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));
    chassis.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> chassis.drive(
                -MathUtil.applyDeadband(driverController.getRawAxis(1), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getRawAxis(0), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(driverController.getRawAxis(2), OIConstants.kDriveDeadband),
                true, true),
            chassis));

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
  }

  private void configureDefaultCommands() {
    //default, but only runs once
    //TODO: Only enable when robot is tested 
    // new Trigger(()->climber.isHomed).whileFalse(new ClimberGoHome(climber));
    driverController.button(2).onTrue(new ClimberGoHome(climber));
    
    // climber.setDefaultCommand(
    //   new RunCommand(
    //     ()->climber.setPower(driverController.getRawAxis(0)*0.1), 
    //     climber)
    // );

    //TODO : Enable for driving but not testing
    // new Trigger(DriverStation::isEnabled)
    // .and(()->climber.isHomed==false)
    // .whileTrue(new ClimberGoHome(climber));  //TODO: disallowed until climber is configured

    leds.setDefaultCommand(leds.showTeamColor());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureDriverBindings() {
    // // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // // cancelling on release.
    // driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    // driverController.x().whileTrue(
    //   new VisionTurnToTargetAprilTag(shooterVision, intakeVision, chassis, navx)
    // );

    //Reset Gyro
    driverController.b().onTrue(new InstantCommand()
    .andThen(new InstantCommand(()-> chassis.zeroHeading(), chassis)));

    operatorJoystick.button(2). //press down button 2 while moving joystick to move
    whileTrue(
      dunkArm.getCommandMoveArmManually(-0.25* operatorJoystick.getRawAxis(1))
    );
    // driverController.a().onTrue(new InstantCommand()
    //   .andThen(new SetShooterProfiled(20, shooter)) //once we get the setAngle working
    // );

    operatorJoystick.button(1)
    .whileTrue(
      new RunCommand(
        ()->climber.setPower(-0.25 * operatorJoystick.getRawAxis(1)), 
        climber)
        .finallyDo(()->climber.setPower(0))
    )
    ;

    new Trigger(passthrough::isBlocked).onTrue(leds.showNoteIntake());
  }

  private void configureOperatorBindings(){
    // operatorJoystick.button(1).whileTrue(new InstantCommand());
    operatorJoystick.button(3).onTrue(new LightingProgressBar(leds, Color.kBlack, Color.kBlue, 5));
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
}
