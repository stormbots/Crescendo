// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class CalibrateShooter extends Command {
  /** Creates a new calibrateShooter. */

  Shooter shooter;
  double startTime = 0.0;

  public CalibrateShooter(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    var config = new SparkMaxConfig(); 
    config.closedLoop.p(0.1,ClosedLoopSlot.kSlot1);
    shooter.shooterMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters.kNoPersistParameters);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.shooterMotor.set(0.04);
    // shooter.shooterMotor.getPIDController().setReference(0.5, ControlType.kCurrent,1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      shooter.syncEncoders();
      shooter.isHomed = true;
      shooter.stopShooter();
      // shooter.shooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 1000);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp()-startTime>0.1) {
      return true;
    }
    return false;
  }
}
