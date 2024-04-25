// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lighting;

import java.util.concurrent.ThreadLocalRandom;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Leds;

public class LightingLightningUniform extends Command {
  double percentOutput;
  double scaleValue;
  Leds leds;
  Color color;
  Leds.HSVColor hsvColor;
  boolean finished;
  int value;
  double startTime;
  double currentTime;
  double elapsedTime;
  double brightTime;
  double darkTime;
  double colorValue;
  /** Creates a new LightingFlicker. */
  public LightingLightningUniform(Leds leds, Color color, double brightness) {
    this.leds = leds;
    this.color = color;
    this.percentOutput = brightness;
    hsvColor = leds.new HSVColor(color);
    scaleValue = brightness/100;
    colorValue = hsvColor.value/255;
    addRequirements(leds);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leds.setColor(color, 0);
    value = (int)Math.round((ThreadLocalRandom.current().nextInt(25, 200))*scaleValue*colorValue);
    startTime = leds.getTime();
    brightTime = (Math.random()+.5)/2;
    darkTime = Math.random()+.5;
    finished = false;

   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentTime = leds.getTime();
    elapsedTime = currentTime-startTime;
    value = (int)Math.round(((ThreadLocalRandom.current().nextInt(0, 6))*51)*scaleValue*colorValue);
    
    if (elapsedTime >= brightTime){
      value = 0;
    }

    if (elapsedTime >= darkTime+brightTime){
      startTime = leds.getTime();
      darkTime = Math.random()+.5;
      brightTime = (Math.random()+.5)/3;
    }

    
    for(var i = 0; i < leds.ledBuffer.getLength(); i++){
      leds.setColor(color, value);
      // SmartDashboard.putNumber("leds/value", value);
      // SmartDashboard.putNumber("leds/startTime", startTime);
      // SmartDashboard.putNumber("leds/currentTime", currentTime);
      // SmartDashboard.putNumber("leds/elapsedTime", elapsedTime);
      // SmartDashboard.putNumber("leds/brightTime", brightTime);
      // SmartDashboard.putNumber("leds/darkTime", darkTime);
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
