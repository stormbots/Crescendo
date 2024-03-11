// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lighting;

import java.util.concurrent.ThreadLocalRandom;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Leds;

public class LightingLightningRandom extends Command {
  double percentOutput;
  double scaleValue;
  Leds leds;
  Color color;
  Leds.HSVColor hsvColor;
  boolean finished;
  int value;
  double[] startTime;
  double currentTime;
  double elapsedTime;
  double[] brightTime;
  double[] darkTime;
  double colorValue;
  double[] loop;
  double timeScale;
  /** Creates a new LightingFlicker. */
  public LightingLightningRandom(Leds leds, Color color, double timeScale, double brightness) {
    this.leds = leds;
    this.color = color;
    this.percentOutput = brightness;
    hsvColor = leds.new HSVColor(color);
    scaleValue = brightness/100;
    colorValue = hsvColor.value/255;
    startTime = new double[leds.ledBuffer.getLength()];
    darkTime = new double[leds.ledBuffer.getLength()];
    brightTime = new double[leds.ledBuffer.getLength()];
    loop = new double[leds.ledBuffer.getLength()];
    this.timeScale =timeScale;
    addRequirements(leds);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leds.setColor(color, 0);
  
    finished = false;
    for(var i = 0; i < leds.ledBuffer.getLength(); i++){
      startTime[i] = leds.getTime();//+ (Math.random());
      brightTime[i] = (Math.random()+.5)/3;
      darkTime[i] = Math.random()+.5;
      loop[i] = 0;
    }

   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    for(var i = 0; i < leds.ledBuffer.getLength(); i++){
      value = 0;
      currentTime = leds.getTime();
      elapsedTime = currentTime-startTime[i];
      if (elapsedTime <= brightTime[i] && loop[i]%timeScale ==0){
        value = (int)Math.round(((ThreadLocalRandom.current().nextInt(0, 256)))*scaleValue*colorValue);
      }

      if (elapsedTime > brightTime[i] && elapsedTime < darkTime[i]+brightTime[i]){
        value = 0;
      }

      if (elapsedTime >= darkTime[i]+brightTime[i]){
        startTime[i] = leds.getTime();
        darkTime[i] = Math.random()+.5;
        brightTime[i] = (Math.random()+.5)/3;
      }
      leds.ledBuffer.setHSV(i, hsvColor.hue, hsvColor.saturation, value);
      loop[i] += 1;
    }
    
     // SmartDashboard.putNumber("leds/value", value);
      // SmartDashboard.putNumber("leds/startTime", startTime);
      // SmartDashboard.putNumber("leds/currentTime", currentTime);
      // SmartDashboard.putNumber("leds/elapsedTime", elapsedTime);
      // SmartDashboard.putNumber("leds/brightTime", brightTime);
      // SmartDashboard.putNumber("leds/darkTime", darkTime);
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
