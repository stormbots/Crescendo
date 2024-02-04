// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LEDs extends SubsystemBase {
  public AddressableLED ledStrip;
  public AddressableLEDBuffer ledBuffer;
  private int rainbowStart;
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
  }

  @Override
  public void periodic() {
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

  public void setLedHSV(int hue, int saturation, int value){
    for(var i = 0; i < ledBuffer.getLength(); i++){
      ledBuffer.setHSV(i, hue, saturation, value);
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

    public Command showTeamColor(){
      return new RunCommand(()->{
        var color = DriverStation.getAlliance();
        if (color.isPresent()){
          if (color.get() == DriverStation.Alliance.Red){
            this.setLedRGBLib(Color.kRed);
          }
          if (color.get() == DriverStation.Alliance.Blue){
            this.setLedRGBLib(Color.kBlue);
          }
          return;
        }
        this.setLedRGBLib(Color.kPurple);
      },this)
      .ignoringDisable(true)
      ;
    }

    public Command showNoteIntake(){
      return new RunCommand(()->this.setLedRGBLib(Color.kOrangeRed),this)
      .withTimeout(2)
      ;
    }

  }

