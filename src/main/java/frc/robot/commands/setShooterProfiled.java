// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Shooter;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class setShooterProfiled extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter shooter;
  private double shooterAngle;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(0, 0);
  TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);
  TrapezoidProfile.State initial = new TrapezoidProfile.State(0, 0);
  TrapezoidProfile shooterProfile = new TrapezoidProfile(constraints);
  double startTimer = 0;

  public setShooterProfiled(double shooterAngle, Shooter shooter) {
    this.shooterAngle = shooterAngle;
    this.shooter = shooter;
    addRequirements(shooter);
    goal = new TrapezoidProfile.State(shooterAngle, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTimer = Timer.getFPGATimestamp();
    initial = new TrapezoidProfile.State(shooter.getShooterAngle(), shooter.shooterMotor.getEncoder().getVelocity());
    shooterProfile = new TrapezoidProfile(constraints);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var targetPosition = shooterProfile.calculate(Timer.getFPGATimestamp()-startTimer, initial, goal).position;
    shooter.setShooterPID(targetPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
