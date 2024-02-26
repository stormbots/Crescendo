// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lighting;

import java.util.concurrent.ThreadLocalRandom;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Leds;

public class LightingPulse extends Command {
  double brightness;
  double scaleValue;
  Leds leds;
  Leds.HSVColor color;
  boolean finished;
  int value;
  double startTime;
  double currentTime;
  double elapsedTime;
  double cycleTime;
  double frequency;
  /** Creates a new LightingFlicker. */
  public LightingPulse(Leds leds, Color color, double cycleTime, double brightness) {
    this.leds = leds;
    this.color = leds.new HSVColor(color);
    this.brightness = brightness;
    scaleValue = brightness/100;
    addRequirements(leds);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leds.setColor(Color.fromHSV(color.hue, color.saturation, color.value), (int)brightness);
    value = (int)(color.value*scaleValue);
    startTime = leds.getTime();
    finished = false;
    frequency = 2/cycleTime;

   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentTime = leds.getTime();
    elapsedTime = currentTime-startTime;
    value = (int)((.5*Math.sin(frequency*elapsedTime*Math.PI)+.5)*(color.value*scaleValue));
   

    
    for(var i = 0; i < leds.ledBuffer.getLength(); i++){
      leds.setColor(Color.fromHSV(color.hue, color.saturation, color.value), value);
      // SmartDashboard.putNumber("leds/value", value);
      // SmartDashboard.putNumber("leds/startTime", startTime);
      // SmartDashboard.putNumber("leds/currentTime", currentTime);
      // SmartDashboard.putNumber("leds/elapsedTime", elapsedTime);
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
