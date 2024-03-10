// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkBase.IdleMode;
import com.stormbots.LUT;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Clamp;
import frc.robot.subsystems.DunkArm;
import frc.robot.subsystems.DunkArmRoller;

public class DunkArmRollerHoldNote extends Command {
  DunkArm dunkarm;
  DunkArmRoller roller;
  double target = 0;

  LUT lut = new LUT(new double[][]{
    {-25,1.7}, //out to clear rollers
    {-10,1.7},
    {0,-4.8}, //in to clear chains
    {60,-4.8},
    {90,2.0} //out to clear wall
  });


  /** Creates a new DunkArmRollerHoldNote. */
  public DunkArmRollerHoldNote(DunkArm dunkarm,DunkArmRoller roller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.dunkarm = dunkarm; 
    this.roller = roller; 
    addRequirements(roller);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //brake
    roller.setIdleMode(IdleMode.kBrake);
    roller.enableSoftLimit(true);
    //soft  limits
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //monitor Dunk Arm Position
    double angle = dunkarm.getAngle();
    double position = roller.getPosition();
    //get ideal position for note at that angle
    if(!Clamp.bounded(position, -5, 16)){ 
      roller.stop();
      return;
    }    

    target = lut.get(angle)[0];
    roller.setPosition(target);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //undo limits
    //coast
    roller.setIdleMode(IdleMode.kCoast);
    roller.enableSoftLimit(false);
    roller.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
