// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.concurrent.ThreadLocalRandom;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs;

public class LightingFlickerUniform extends Command {
  double percentOutput;
  double scaleValue;
  LEDs leds;
  Color color;
  boolean finished;
  int[] hsv;
  int value;
  double startTime;
  double currentTime;
  double elapsedTime;
  /** Creates a new LightingFlicker. */
  public LightingFlickerUniform(LEDs leds, Color color, double percentOutput) {
    this.leds = leds;
    this.color = color;
    this.percentOutput = percentOutput;
    scaleValue = percentOutput/100;
    addRequirements(leds);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   hsv = leds.rgbToHsv(color);
   value = (int)Math.round((ThreadLocalRandom.current().nextInt(25, 200))*scaleValue);
   startTime = leds.getTime();
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentTime = leds.getTime();
    elapsedTime = currentTime-startTime;
    if (elapsedTime <= (Math.random()/10)){
      value = (int)Math.round((ThreadLocalRandom.current().nextInt(0, 11))*25*scaleValue);
      
    }
    else if (elapsedTime >= (Math.random()*5)){
      value = 0;
      startTime = leds.getTime();
    }
    
    for(var i = 0; i < leds.ledBuffer.getLength(); i++){
      leds.ledBuffer.setHSV(i, hsv[0], hsv[1], value);
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
