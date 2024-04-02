// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterFlywheel;

public class SetFlywheelSlew extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private ShooterFlywheel flywheel;

  SlewRateLimiter flywheelRateLimiter =
      new SlewRateLimiter(
          ShooterFlywheel.kSlewForward, ShooterFlywheel.kSlewBackward, 0); // TODO: get rate limits
  double targetRPM = 0.0;
  boolean exitsOnCompletion = true;

  public SetFlywheelSlew(double targetRPM, ShooterFlywheel flywheel) {
    this.flywheel = flywheel;
    this.targetRPM = targetRPM;

    addRequirements(flywheel);
  }

  public SetFlywheelSlew runForever() {
    this.exitsOnCompletion = false;
    return this;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    flywheelRateLimiter.reset(flywheel.getRPM());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var target = flywheelRateLimiter.calculate(targetRPM);
    flywheel.setRPM(target);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      flywheel.setRPM(targetRPM);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return exitsOnCompletion && flywheel.isOnTarget();
  }
}
