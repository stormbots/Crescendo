// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs;

  
public class LightingProgressBarFlow extends Command {
  /** Creates a new LightingColor. */
  LEDs leds;
  Color backGroundColor;
  Color progressColor;
  double timeLimt;
  double startTime;
  boolean finished;
  double percentOutput;
  int[] value;
  double percent;
  int index;


  public LightingProgressBarFlow(LEDs leds, Color backGroundColor, Color progressColor, double timeLimt, double percentOutput) {
    this.leds = leds;
    this.backGroundColor = backGroundColor;
    this.progressColor = progressColor;
    this.timeLimt = timeLimt;
    this.percentOutput = percentOutput;
    value = new int[leds.ledBuffer.getLength()];// (int)Math.round((255*(percentOutput/100)));

    addRequirements(leds);
  }
    // Use addRequirements() here to declare subsystem dependencies.
    

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    finished = false;
    // leds.ledStrip.setData(leds.ledBuffer);
    // leds.ledStrip.start();
    startTime = leds.getTime();
    leds.setLedRGB(backGroundColor);
    SmartDashboard.putNumber("leds/startTime", startTime);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    var currentTime = leds.getTime();
    SmartDashboard.putNumber("leds/currentTime", currentTime);
    var elapsedTime = currentTime-startTime;
    SmartDashboard.putNumber("leds/elapsedTime", elapsedTime);
    double timePerLED = (timeLimt / leds.ledBuffer.getLength());
    SmartDashboard.putNumber("timePerLed", timePerLED);
    double numberOfChannelsOn = (double)(elapsedTime / timePerLED);
    SmartDashboard.putNumber("leds/channels", numberOfChannelsOn);
    if(numberOfChannelsOn > leds.ledBuffer.getLength()){
      finished = true;
      numberOfChannelsOn = leds.ledBuffer.getLength();
    }

    int[] hsv = leds.rgbToHsv(progressColor);
    int hue = hsv[0];
    int saturation = hsv[1];
    

    for(var i = 0; i < numberOfChannelsOn; i++){
      value[i] = 0;
      if (i < numberOfChannelsOn) {
        
        value[i] = (int)Math.round((255*(percentOutput/100)));
      }
      
      if (i == (int)numberOfChannelsOn){
        percent = (((elapsedTime%timePerLED)/timePerLED));
        value[i] = (int)Math.round((percent*(255*(percentOutput/100))));

      }
      if(i< numberOfChannelsOn){
        index = i;
      }
      if(i >= numberOfChannelsOn){
        index = i;
      }
      leds.ledBuffer.setHSV(index, hue, saturation, value[i]);
      SmartDashboard.putNumber("leds/value", value[i]);
      SmartDashboard.putNumber("leds/percent", percent*100);
      SmartDashboard.putNumber("leds/percentTime", elapsedTime%timePerLED);
    }
  }
    // if (channels + 1 > leds.ledBuffer.getLength()){
    //   finished = true;
    // }
    //do any time math
    //set color

  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leds.setLedRGB(backGroundColor);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}


