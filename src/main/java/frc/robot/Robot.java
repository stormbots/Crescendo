// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import com.stormbots.Lerp;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Chassis.ChassisConstants.DriveConstants;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer robotContainer;

  public static boolean isCompbot = true;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    var entry = SmartDashboard.getEntry("isCompbot");
    isCompbot = entry.getBoolean(true);
    entry.setBoolean(isCompbot);
    entry.setPersistent();

    // set the value of chassis track width based on bot
    //TODO: Get rid of this
    // DriveConstants.kTrackWidth =
    //     isCompbot ? Units.inchesToMeters(23.5) : Units.inchesToMeters(24.5);
    // DriveConstants.kWheelBase = isCompbot ? Units.inchesToMeters(23.5) : Units.inchesToMeters(24.5);
    // DriveConstants.distanceToModuleFromCenter =
    //     Math.hypot(DriveConstants.kTrackWidth / 2, DriveConstants.kWheelBase / 2);

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putData(CommandScheduler.getInstance());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    robotContainer.chassis.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    robotContainer.climber.setPower(0); // Prevent surprise climber motion
    // new RunCommand(()->robotContainer.dunkArm.setPower(0), robotContainer.dunkArm).schedule();
    // robotContainer.chassis.zeroHeading();
    // robotContainer.chassis.resetOdometry(new Pose2d());
    // robotContainer.chassis.resetEncoders();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    robotContainer.climber.setPower(0); // Prevent surprise climber motion
    // robotContainer.shooterVision.setPipeline(ShooterVision.LimelightPipeline.kSpeaker);
    // robotContainer.climber.setIdleMode(IdleMode.kBrake);

    // robotContainer.chassis.resetOdometry(new Pose2d());
    // robotContainer.chassis.resetEncoders();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // check slider
    // map slider from initial to initial+2
    RobotContainer.shooterOffset =
        Lerp.lerp(
            robotContainer.operatorJoystick.getRawAxis(3),
            1,
            -1,
            RobotContainer.INITIALSHOOTEROFFSET,
            RobotContainer.INITIALSHOOTEROFFSET + 5);
    SmartDashboard.putNumber("shooterOffset", RobotContainer.shooterOffset);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

    robotContainer.climber.setPower(0); // Prevent surprise climber motion
    // robotContainer.chassis.setIdleMode(IdleMode.kCoast);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
