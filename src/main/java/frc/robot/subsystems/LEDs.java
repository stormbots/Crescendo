// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.TickMeter;

import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LEDs extends SubsystemBase {
  public AddressableLED ledStrip;
  public AddressableLEDBuffer ledBuffer;
  private int rainbowStart;
  private int loadStart;
  /** Creates a new LEDs. */
  public LEDs() {
    ledStrip = new AddressableLED(9);
    ledBuffer = new AddressableLEDBuffer(16);
    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();
    
    rainbowStart = 0;
    loadStart = 0;

  }

  @Override
  public void periodic() {
    //rainbow(1);
    //setLedRGBLib(Color.kPurple);
    //flickerUniform();
    ledStrip.setData(ledBuffer);
    
    
    // This method will be called once per scheduler run
  }

  public void setLedHSV(int hue, int saturation, int value){
    for(var i = 0; i < ledBuffer.getLength(); i++){
      ledBuffer.setHSV(i, hue, saturation, value);
      // ledBuffer.setHSV(i, 0, 100, 100);
    }
  }
  public void setLedRGB(int red, int green, int blue){
    for(var i = 0; i < ledBuffer.getLength(); i++){
      ledBuffer.setRGB(i, red, green, blue);
    }
  }

  public void setLedRGBLib(Color color){
    int red = (int)(color.red*255);
    int green = (int)(color.green*255);
    int blue = (int)(color.blue*255);
    for(var i = 0; i < ledBuffer.getLength(); i++){
      ledBuffer.setRGB(i, red, green, blue);
    }
    SmartDashboard.putNumber("LED/Red", red);
    SmartDashboard.putNumber("LED/Green",green);
    SmartDashboard.putNumber("LED/Blue",blue);
    SmartDashboard.putNumber("LED/Red Raw", color.red);
    SmartDashboard.putNumber("LED/Green Raw",color.green);
    SmartDashboard.putNumber("LED/Blue Raw",color.blue);
  }

  public void rainbow(double speed){
    for(var i = 0;  i < ledBuffer.getLength();i++){
      final var hue = (rainbowStart + (i*180/ledBuffer.getLength())) %180;
      ledBuffer.setHSV(i, hue, 255, 25);
    }
    rainbowStart += speed;
    rainbowStart %= 180;
    }
  //load doesn't work
  public void load(double startTime){
    var currentTime = Timer.getFPGATimestamp();
    var elapsedTime = currentTime-startTime;
    var maxTime = 5;
    var timePerChannel = maxTime / ledBuffer.getLength();
      
      var chs = (int) (elapsedTime / timePerChannel);
         for(var i =0; i < chs; i++){
          int value = 50;
          ledBuffer.setHSV(i, 0, 255, value);
        }
      
      } 
  }

  // public void flickerUniform(){
  //   int value = 
  //   setLedHSV(0, 255, value);
  // }

  

