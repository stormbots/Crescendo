// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/** An example command that uses an example subsystem. */
public class SetShooterProfiled extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Shooter shooter;
  private double shooterAngle;
  private Boolean exitsOnCompletion = true;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  // TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(360*8, 180*8*2/5);
  // TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);
  // TrapezoidProfile.State initial = new TrapezoidProfile.State(0, 0);
  // TrapezoidProfile shooterProfile = new TrapezoidProfile(constraints);
  // double startTimer = 0;
  double targetPosition = 0.0;

  SlewRateLimiter shooterRateLimiter = new SlewRateLimiter(
  Shooter.kSlewForward, Shooter.kSlewBackward, 0); //TODO: get rate limits

    
  public SetShooterProfiled(double shooterAngle, Shooter shooter) {
    this.shooterAngle = shooterAngle;
    this.shooter = shooter;
    addRequirements(shooter);
    // goal = new TrapezoidProfile.State(shooterAngle, 0);
  }

  public SetShooterProfiled withConstrants(TrapezoidProfile.Constraints constraints){
    //set the value
    // this.constraints = constraints;
    // this.shooterProfile = new TrapezoidProfile(constraints);

    return this;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // startTimer = Timer.getFPGATimestamp();
    // initial = shooter.getState();
    // shooterProfile = new TrapezoidProfile(constraints);

    shooterRateLimiter.reset(shooter.getShooterAngle());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // var currentState =shooter.getState();

    // targetPosition = shooterProfile.calculate(Timer.getFPGATimestamp()-startTimer, currentState, goal).position;
    // shooter.setAngle(targetPosition);
    // SmartDashboard.putNumber("profile/target", targetPosition);

    var target = shooterRateLimiter.calculate(shooterAngle);
    shooter.setAngle(target);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Will need to figure out, add feedforward stuff?
    SmartDashboard.putBoolean("shooter/interrupted", interrupted);
    // if(interrupted == false){
    //   shooter.setAngle(shooterAngle);
    // }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  
    return exitsOnCompletion && shooter.isOnTarget(shooterAngle);
  }

  public SetShooterProfiled runForever(){
    this.exitsOnCompletion = false;
    return this;
  }
}
