// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.opencv.core.TickMeter;

import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;


public class LEDs extends SubsystemBase {
  private java.awt.Color javaColorLib;
  public AddressableLED ledStrip;
  public AddressableLEDBuffer ledBuffer;
  private int rainbowStart;
  private int loadStart;
  double startTime;
  /** Creates a new LEDs. */
  public LEDs() {
    ledStrip = new AddressableLED(9);
    ledBuffer = new AddressableLEDBuffer(16);
    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();
    startTime = 0;
    rainbowStart = 0;
    loadStart = 0;
    

  }

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("startTime", getStartTime());
    
    //rainbow(1);
    // setLedRGBLib(Color.kCyan);
    //flickerUniform();
    ledStrip.setData(ledBuffer);
    
    
    // This method will be called once per scheduler run
  }

  public void setBuffer(AddressableLEDBuffer buffer){
    ledStrip.setData(buffer);
  }

  public double getStartTime(){
    startTime = Timer.getFPGATimestamp();
    return startTime;
  }
  //DOES NOT WORK
  // public int[] rgbToHsv(Color color){
  //   int r = (int)color.red*255;
  //   int g = (int)color.green*255;
  //   int b = (int)color.blue;
  //   float[] hsvFloat = new float[3];
  //   java.awt.Color.RGBtoHSB(r,g,b,hsvFloat);
  //   int[] hsv = new int[3];
  //   hsv[0] = (int)hsvFloat[0]*180;
  //   hsv[1] = (int)hsvFloat[1]*255;
  //   hsv[2] = (int)hsvFloat[2]*255;
  //   return hsv;
  // }

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
  }

  public void setLedRGBLib(Color color, double channels){
    int red = (int)(color.red*255);
    int green = (int)(color.green*255);
    int blue = (int)(color.blue*255);
    for(var i = 0; i < channels; i++){
      ledBuffer.setRGB(i, red, green, blue);
    }
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
  public void load(double timeLimt, Color color){
    var currentTime = Timer.getFPGATimestamp();
    var elapsedTime = currentTime-startTime;
    double maxTime = timeLimt;
    double timePerChannel = (maxTime / ledBuffer.getLength());  
    var chs = (int)(Math.round((elapsedTime / timePerChannel)));
    if(chs > ledBuffer.getLength()){
      chs = ledBuffer.getLength();
    }
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

  

