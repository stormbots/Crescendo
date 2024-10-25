// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lighting;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Leds;

  
public class LightingProgressBarFlow extends Command {
  /** Creates a new LightingColor. */
  Leds leds;
  Color backGroundColor;
  Leds.HSVColor progressColor;
  double timeLimt;
  double startTime;
  boolean finished;
  int brightness;
  int[] value;
  double percent;

  public LightingProgressBarFlow(Leds leds, Color backGroundColor, Color progressColor, double timeLimt, double brightness) {
    this.leds = leds;
    this.backGroundColor = backGroundColor;
    this.progressColor = leds.new HSVColor(progressColor);
    this.timeLimt = timeLimt;
    this.brightness = (int)Math.round(brightness);
    value = new int[leds.ledBuffer.getLength()];

    addRequirements(leds);
  }
    // Use addRequirements() here to declare subsystem dependencies.
    

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    finished = false;
    startTime = leds.getTime();
    leds.setColor(backGroundColor,brightness);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    var currentTime = leds.getTime();
    var elapsedTime = currentTime-startTime;
    double timePerLED = (timeLimt / leds.ledBuffer.getLength());
    double numberOfChannelsOn = (double)(elapsedTime / timePerLED);
    if (numberOfChannelsOn > leds.ledBuffer.getLength()){
      numberOfChannelsOn = leds.ledBuffer.getLength();
    }
    if(elapsedTime > timeLimt){
      finished = true;
    }

    // int[] hsv = leds.rgbToHsv(progressColor);
    // int hue = hsv[0];
    // int saturation = hsv[1];
    
    for(var i = 0; i < numberOfChannelsOn; i++){
      value[i] = 0;
      if (i < numberOfChannelsOn) {
        
        value[i] = (int)Math.round(progressColor.value*brightness/100.0);;
      }
      
      if (i == (int)numberOfChannelsOn){
        percent = (((elapsedTime%timePerLED)/timePerLED));
        value[i] = (int)Math.round((percent*(progressColor.value*brightness/100.0)));
      }
      
      leds.ledBuffer.setHSV(i, progressColor.hue, progressColor.saturation, value[i]);

      // SmartDashboard.putNumber("leds/value", value[i]);
      // SmartDashboard.putNumber("leds/percent", percent*100);
      // // SmartDashboard.putNumber("leds/percentTime", elapsedTime%timePerLED);
      // SmartDashboard.putNumber("leds/channels", numberOfChannelsOn);
      // // SmartDashboard.putNumber("timePerLed", timePerLED);
      // // SmartDashboard.putNumber("leds/elapsedTime", elapsedTime);
      // // SmartDashboard.putNumber("leds/currentTime", currentTime);
      // // SmartDashboard.putNumber("leds/startTime", startTime);

    }
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leds.setColor(backGroundColor, brightness);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}


