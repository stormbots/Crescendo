// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Passthrough;

public class IntakeNote extends Command {
  
  private Intake intake;
  private Passthrough passthrough;
  boolean exitsOnCompletion = true;

  /** Creates a new IntakeNote. */
  public IntakeNote(Intake intake, Passthrough passthrough) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake; 
    this.passthrough = passthrough;
    addRequirements(intake,passthrough);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.intake();
    passthrough.setPower(0.1);

    if (exitsOnCompletion==false) {
      passthrough.intake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
    intake.stop();
    passthrough.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return exitsOnCompletion && (intake.isBlocked() || passthrough.isBlocked());
  }

  public IntakeNote runForever(){
    this.exitsOnCompletion = false;
    return this;
  }
}
