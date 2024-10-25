// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lighting;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Leds;

public class LightingRainbowSolid extends Command {
  Leds leds;
  double secondsPerCycle;
  double startHue = 0;
  double percentOutput;
  int value;
  double startTime;
  boolean finished;
  /** Creates a new LightingRainbow. */
  public LightingRainbowSolid(Leds leds, double secondsPerCycle, double percentOutput) {
    this.leds = leds;
    this.secondsPerCycle = secondsPerCycle;
    this.percentOutput = percentOutput;
    value = (int)Math.round((255*(percentOutput/100)));
    addRequirements(leds);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = leds.getTime();
    finished = false;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentTime = leds.getTime();
    double elapsedTime = currentTime - startTime;
    double timePerColor = secondsPerCycle / 180;
    
    for(var i = 0;  i < leds.ledBuffer.getLength();i++){
      int hue = (int)Math.round(startHue % 180);
      leds.ledBuffer.setHSV(i, hue, 255, value);
    }
    if (secondsPerCycle > 0){
      this.startHue += ((elapsedTime)/timePerColor);
      this.startHue %= 180;
      this.startTime = leds.getTime();
    }
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    finished = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }

}
