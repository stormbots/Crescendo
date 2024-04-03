// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShooterVision;

public class DriverFeedback extends Command {
  /** Creates a new DriverFeedback. */

  ShooterVision vision;
  CommandXboxController driverController;
  BooleanSupplier[] booleans;
  double startTimer = 0.0;

  public DriverFeedback(ShooterVision vision, CommandXboxController driverController, BooleanSupplier ... conditions) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.vision = vision;
    this.driverController = driverController;
    booleans = conditions;
    //this.ignoringDisable(true);
  }

  public DriverFeedback(CommandXboxController driverController, BooleanSupplier ... conditions) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driverController = driverController;
    booleans = conditions;
    //this.ignoringDisable(true);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTimer = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    for (BooleanSupplier condition : booleans) {
      if (!condition.getAsBoolean()) {
        driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
        return;
      }
    }
    driverController.getHID().setRumble(RumbleType.kBothRumble, 0.5);

    double ledTimer = Timer.getFPGATimestamp();
    if (vision != null) {
      if (ledTimer-startTimer<0.3) vision.setLEDMode(2.0);
      else vision.setLEDMode(1.0);
    }
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    if (vision!=null) vision.setLEDMode(1.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
