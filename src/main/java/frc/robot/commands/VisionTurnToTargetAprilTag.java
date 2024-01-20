// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.FrontVision;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Vision.LimelightPipeline;

import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class VisionTurnToTargetAprilTag extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private Vision vision;
  private Chassis chassis;
  private AHRS navx;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public VisionTurnToTargetAprilTag(Vision vision, Chassis chassis, AHRS navx) {
    this.vision = vision;
    this.chassis = chassis;
    this.navx = navx;
    //TODO: get turn power/pid/field stuff

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision);
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    vision.setPipeline(Vision.LimelightPipeline.kNoZoom);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<Double> distance = vision.getDistanceAprilTag();
    Optional<Double> angle; //TODO: use bot pose to get Translation2d to target
    //TODO: actually turning!!

    SmartDashboard.putNumber("limelight/distance", distance.get());
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