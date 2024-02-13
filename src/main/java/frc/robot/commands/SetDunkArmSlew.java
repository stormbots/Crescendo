// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DunkArm;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class SetDunkArmSlew extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private double angle;
  DunkArm dunkArm;
  SlewRateLimiter armRateLimiter =new SlewRateLimiter(
    90, -90, -20); //TODO: get rate limits
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public SetDunkArmSlew(double angle, DunkArm dunkArm) {
    this.angle = angle;
    this.dunkArm = dunkArm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(dunkArm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armRateLimiter.reset(dunkArm.getAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var target = armRateLimiter.calculate(angle);
    dunkArm.setArmAngle(target);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dunkArm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
