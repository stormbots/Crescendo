// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimberGoHome extends Command {
  /** Creates a new ClimberGoHome. */

  Climber climber;
  public ClimberGoHome(Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    climber.setPower(climber.kHomePower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
    climber.setPower(0);
    if(interrupted) return; //not homed!!
    climber.setHomed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //what we care about: Current home threshhold;
    return climber.isAtHomePosition();
  }
}
