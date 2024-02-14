// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BlinkinLeds;


public class Leds extends SubsystemBase {
  public AddressableLED ledStrip;
  public AddressableLEDBuffer ledBuffer;
  boolean hasRun;
  BlinkinLeds blinkinLeds;
  /** Creates a new LEDs. */
  public Leds() {
    ledStrip = new AddressableLED(9);
    ledBuffer = new AddressableLEDBuffer(16);
    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();
    hasRun = false;
    
  }

  @Override
  public void periodic() {
    ledStrip.setData(ledBuffer);

    // This method will be called once per scheduler run
  }

  
  public double getTime(){
    double time = Timer.getFPGATimestamp();
    return time;
  }

 public int[] rgbToHsv(Color color){
  double r = color.red;
  double g = color.green;
  double b = color.blue;
  double max = Math.max(r, Math.max(g, b)); // maximum of r, g, b 
  double min = Math.min(r, Math.min(g, b)); // minimum of r, g, b 
  double range = max - min; // diff of cmax and cmin. 
  double h = -1, s = -1; 
  if (max == min) 
    h = 0; 
  else if (max == r) 
    h = ((60 * ((g - b) / range) + 360) % 360)/2; 
  else if (max == g) 
    h = ((60 * ((b - r) / range) + 120) % 360)/2; 
  else if (max == b) 
    h = ((60 * ((r - g) / range) + 240) % 360)/2; 
  if (max == 0) 
    s = 0; 
  else
    s = (range / max) * 255; 
  double v = max * 255; 
  int[] hsv = new int[3];
  hsv[0] = (int)Math.round(h);
  hsv[1] = (int)Math.round(s);
  hsv[2] = (int)Math.round(v);
  return hsv;
 }

  private void setLedHsvManual(int hue, int saturation, int value){
    for(var i = 0; i < ledBuffer.getLength(); i++){
      ledBuffer.setHSV(i, hue, saturation, value);
    }
  }

  private void setLedRgbManual(int red, int green, int blue){
    for(var i = 0; i < ledBuffer.getLength(); i++){
      ledBuffer.setRGB(i, red, green, blue);
    }
  }

  public int matchBrightnessScaling(int disabledBrightness, int enabledBrightness){
    if (DriverStation.isDisabled()){
      return disabledBrightness;
    }
    return enabledBrightness;
  }

  public void setLedRGB(Color color){
    int red = (int)(color.red*255);
    int green = (int)(color.green*255);
    int blue = (int)(color.blue*255);
    for(var i = 0; i < ledBuffer.getLength(); i++){
      ledBuffer.setRGB(i, red, green, blue);
    }
  }

  public void setLedRGB(Color color, double channels){
    int red = (int)(color.red*255);
    int green = (int)(color.green*255);
    int blue = (int)(color.blue*255);
    for(var i = 0; i < channels; i++){
      ledBuffer.setRGB(i, red, green, blue);
    }
  }

  public void setLedHSV(Color color, int percentOutput){
    int[] hsv = rgbToHsv(color);
    int hue = hsv[0];
    int saturation = hsv[1];
    int value = (int) Math.round((hsv[2]*percentOutput/100));
    for(var i = 0; i < ledBuffer.getLength(); i++){
      ledBuffer.setHSV(i, hue, saturation, value);
    }
  }

  public void setLedHSV(Color color, int percentOutput, double channels){
    int[] hsv = rgbToHsv(color);
    int hue = hsv[0];
    int saturation = hsv[1];
    int value = (int) Math.round((hsv[2]*percentOutput/100));
    for(var i = 0; i < channels; i++){
      ledBuffer.setHSV(i, hue, saturation, value);
    }
  }

    private void runOnce(){
      setLedHSV(Color.kGreen, 100);
      hasRun = true;
    }

    public boolean hasRun(){
      if (DriverStation.isEnabled()&& hasRun == false){
        return true;
      }
      return false;
      
    }

    public Command showTeamColor(){
      return new RunCommand(()->{
        var color = DriverStation.getAlliance();
        if (color.isPresent()){
          if (color.get() == DriverStation.Alliance.Red){
            this.setLedHSV(Color.kRed, this.matchBrightnessScaling(10, 100));
          }
          if (color.get() == DriverStation.Alliance.Blue){
            this.setLedHSV(Color.kBlue, this.matchBrightnessScaling(10, 100));
          }
          return;
        }
        this.setLedHSV(Color.kPurple, 10);
      },this)
      .ignoringDisable(true)
      ;
    }

    public Command showNoteIntake(){
      return new RunCommand(()->this.setLedRGB(Color.kOrangeRed),this).withTimeout(2);
    }
    
  }

