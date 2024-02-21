// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Passthrough;

public class PassthroughAlignNote extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Passthrough passthrough;
  private final Intake intake;

  /** Creates a new PassthroughAlignNote. */
  public PassthroughAlignNote(Passthrough passthrough, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.passthrough = passthrough;
    this.intake = intake;

    addRequirements(passthrough);
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Setup PID for the passthroughMotor
    //Have it adjusting within the ideal range for passthrough
    var dist = passthrough.getSensorDistance().in(Units.Inches);
    var nominal = 2;
    //TODO: Adjust these values to more idea placement
    var kpassthrough = 0.1;
    var kintake = 0.1;

    var kpassresponse = (dist-nominal) * kpassthrough;
    var kintakeresponse = (dist-nominal) * kintake;

    passthrough.setPower(kpassresponse);
    intake.setPower(kintakeresponse);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    passthrough.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return passthrough.getSensorDistance().in(Units.Inches) > 6;
  }
}
