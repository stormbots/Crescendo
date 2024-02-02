// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.FieldPosition;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.IntakeVision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterVision;

import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class VisionTurnToTargetOdometry extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private ShooterVision shooterVision;
  private IntakeVision intakeVision;
  private Chassis chassis;
  private AHRS navx;
  private FieldPosition.TargetType speaker = FieldPosition.TargetType.Speaker;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public VisionTurnToTargetOdometry(ShooterVision shooterVision, IntakeVision intakeVision, Chassis chassis, AHRS Navx) {  //not accounted for multiple cameras yet!!
    this.shooterVision = shooterVision;
    this.intakeVision = intakeVision;
    this.chassis = chassis;
    this.navx = navx;
    //TODO: get turn power/pid/field stuff

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterVision);
    addRequirements(intakeVision);
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterVision.setPipeline(ShooterVision.LimelightPipeline.kNoZoom);
    intakeVision.setPipeline(IntakeVision.LimelightPipeline.kNoZoom);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = FieldPosition.GetChassisRotationToPoint(intakeVision.botPose, FieldPosition.GetTargetList(speaker), navx);
    var rotation = 0.5/60.0 * x;
    chassis.drive(0, 0, rotation, true, true);
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