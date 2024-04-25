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

public class LightingFlow extends Command {
  double brightness;
  double scaleValue;
  Leds leds;
  Leds.HSVColor hsvColor;
  Color color;
  boolean finished;
  int value;
  double startTime;
  double currentTime;
  double elapsedTime;
  double wavelength;
  double speed;
  double adjustedWavelength;
  double colorValue;
  double offset = 3;
  /** Creates a new LightingFlicker. */
  public LightingFlow(Leds leds, Color color, double wavelength, double speed, double brightness) {
    this.leds = leds;
    this.color = color;
    hsvColor = leds.new HSVColor(color);
    this.brightness = brightness;
    scaleValue = brightness/100;
    this.speed = speed;
    this.wavelength = wavelength;
    colorValue = hsvColor.value/255;
    addRequirements(leds);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leds.setColor(Color.fromHSV(hsvColor.hue,hsvColor.saturation, hsvColor.value), (int)brightness);
    value = (int)(hsvColor.value*scaleValue);
    startTime = leds.getTime();
    finished = false;
    adjustedWavelength = (2/wavelength)*Math.PI;

   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    for(var i = 0; i < leds.ledBuffer.getLength(); i++){
      currentTime = leds.getTime();
      elapsedTime = currentTime-startTime;
      double adjustedElapsedTime = elapsedTime*speed;
      value = (int)((.5*Math.sin(adjustedWavelength*(adjustedElapsedTime-(i)))+.5)*(hsvColor.value*scaleValue));
      value += offset;
      if (value >255){
        value = 255;
      }
      leds.ledBuffer.setHSV(i, hsvColor.hue, hsvColor.saturation, value);
      SmartDashboard.putNumber("leds/value", value);
      SmartDashboard.putNumber("leds/startTime", startTime);
      SmartDashboard.putNumber("leds/currentTime", currentTime);
      SmartDashboard.putNumber("leds/elapsedTime", elapsedTime);
      SmartDashboard.putNumber("leds/sinElapsedTime", (.5*Math.sin(adjustedWavelength*(adjustedElapsedTime+(i/wavelength)))+.5));
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
