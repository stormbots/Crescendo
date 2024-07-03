// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lighting;

import java.util.concurrent.ThreadLocalRandom;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LedLights;

public class LightingPulse extends Command {
  double brightness;
  double scaleValue;
  LedLights leds;
  LedLights.HSVColor hsvColor;
  double colorValue;
  Color color;
  boolean finished;
  int value;
  double startTime;
  double currentTime;
  double elapsedTime;
  double cycleTime;
  double frequency;
  /** Creates a new LightingFlicker. */
  public LightingPulse(LedLights leds, Color color, double cycleTime, double brightness) {
    this.leds = leds;
    this. color = color;
    this.cycleTime = cycleTime;
    this.hsvColor = leds.new HSVColor(color);
    this.brightness = brightness;
    scaleValue = brightness/100;
    colorValue = hsvColor.value/255;
    addRequirements(leds);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    finished = false;
    leds.setColor(color, (int)brightness);
    value = (int)(hsvColor.value*scaleValue);
    startTime = leds.getTime();
    frequency = 2/cycleTime;

   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentTime = leds.getTime();
    elapsedTime = currentTime-startTime;
    double adjustedtime = elapsedTime*frequency*.5;
    value = (int)((.5*Math.sin(adjustedtime*Math.PI)+.5)*(colorValue*brightness));
    
   

    
    for(var i = 0; i < leds.ledBuffer.getLength(); i++){
      leds.setColor(color, value);
      // SmartDashboard.putNumber("leds/value", value);
      // SmartDashboard.putNumber("leds/startTime", startTime);
      // SmartDashboard.putNumber("leds/currentTime", currentTime);
      // SmartDashboard.putNumber("leds/elapsedTime", elapsedTime);
      // SmartDashboard.putNumber("leds/sinElapsedTimeValue", (int)Math.round(((.5*Math.sin(adjustedtime*Math.PI)+.5)*(colorValue*scaleValue))));
      // SmartDashboard.putNumber("leds/sinElapsedTimeValue", (((.5*Math.sin(adjustedtime*Math.PI)+.5))));
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
