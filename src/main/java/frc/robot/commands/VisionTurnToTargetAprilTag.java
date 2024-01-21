// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeVision;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.ShooterVision;

import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class VisionTurnToTargetAprilTag extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private ShooterVision shooterVision;
  private IntakeVision intakeVision;
  private Chassis chassis;
  private AHRS navx;
  

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public VisionTurnToTargetAprilTag(ShooterVision shooterVision, IntakeVision intakeVision, Chassis chassis, AHRS navx) {
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
    Optional<Double> distanceShooter = shooterVision.getDistanceAprilTag();
    Optional<Double> distanceIntake = intakeVision.getDistanceAprilTag();
    if (distanceShooter.isPresent()) {
      double x = shooterVision.getHorizontalOffset();
      double y = shooterVision.getVerticalOffset();
      SmartDashboard.putNumber("shooter/x", x);
      SmartDashboard.putNumber("shooter/y", y);
      
    }
    if (distanceIntake.isPresent()) {}
    //TODO: actually turning!!

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