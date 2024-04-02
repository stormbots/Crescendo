// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeVision;
import frc.robot.subsystems.ShooterVision;

/** An example command that uses an example subsystem. */
public class OdometryWithVision extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private ShooterVision shooterVision;

  private IntakeVision intakeVision;
  private SwerveDrivePoseEstimator poseEstimator;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public OdometryWithVision(
      ShooterVision shooterVision,
      IntakeVision intakeVision,
      SwerveDrivePoseEstimator poseEstimator) { // front vision
    this.shooterVision = shooterVision;
    this.intakeVision = intakeVision;
    this.poseEstimator = poseEstimator;
    // Use addRequirements() here to declare subsystem dependencies
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (shooterVision.getVisibleTargetData().isPresent()) {
      poseEstimator.addVisionMeasurement(
          shooterVision.poseEstimator.getEstimatedPosition(), Timer.getFPGATimestamp());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
