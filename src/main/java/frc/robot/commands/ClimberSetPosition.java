// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberSetPosition extends Command {
  /** Creates a new ClimberSetPosition. */
  Climber climber;
  Measure<Distance> target;
  SlewRateLimiter slewRate= new SlewRateLimiter(15, -12, 0.0);
  TrapezoidProfile motionProfile = new TrapezoidProfile(
    new TrapezoidProfile.Constraints (0.5, 5.0)
    );
    TrapezoidProfile.State initialPos = new TrapezoidProfile.State(0, 0);
    TrapezoidProfile.State goalPos;

  double startTimer = 0;
  

  public ClimberSetPosition(Climber climber, Measure<Distance> target) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    this.target = target;

    goalPos = new TrapezoidProfile.State(target.in(Units.Inches), 0);

    addRequirements(climber);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTimer = Timer.getFPGATimestamp();

    initialPos = new TrapezoidProfile.State(climber.getPosition().in(Units.Inches), 0);
    slewRate.reset(climber.getPosition().in(Units.Inches));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetPosition = motionProfile.calculate(Timer.getFPGATimestamp()-startTimer, initialPos, goalPos).position;
    double slewRatePosition = slewRate.calculate(target.in(Units.Inches));
    climber.setPosition(Units.Inches.of(slewRatePosition));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return climber.isAtSetpoint() && motionProfile.isFinished(startTimer + motionProfile.totalTime());
  }
}
