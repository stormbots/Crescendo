// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.stormbots.BlinkenPattern;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Leds extends SubsystemBase {
  public AddressableLED ledStrip;
  public AddressableLEDBuffer ledBuffer;
  Servo blinkin1 = new Servo(2);
  Servo blinkin2 = new Servo(0);

  public class HSVColor{
    // private Color rgb;
    public int hue;
    public int saturation;
    public int value;
    public HSVColor(int hue, int saturation,int value){
      this.hue = hue;
      this.saturation = saturation;
      this.value = value;
    }
    
    public HSVColor(Color color){
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
      this.hue = (int)Math.round(h);
      this.saturation = (int)Math.round(s);
      this.value = (int)Math.round(v);
    }
  }


  /** Creates a new LEDs. */
  public Leds() {
    ledStrip = new AddressableLED(9);
    ledBuffer = new AddressableLEDBuffer(16);
    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();
    SmartDashboard.putData("leds/prepare", new RunCommand(this::preparing));
    SmartDashboard.putData("leds/ready", new RunCommand(this::ready));
    blinkin1.setBoundsMicroseconds(2125, 1501, 1500, 1499, 1000);   
  }

  @Override
  public void periodic() {
    ledStrip.setData(ledBuffer);
    // This method will be called once per scheduler run
  }
  
  public double getTime() {
    double time = Timer.getFPGATimestamp();
    return time;
  }

  

  public int matchBrightnessScaling(int disabledBrightness, int enabledBrightness) {
    if (DriverStation.isDisabled()){
      return disabledBrightness;
    }
    return enabledBrightness;
  }

  public void setColor(Color color) {
    int red = (int)(color.red*255);
    int green = (int)(color.green*255);
    int blue = (int)(color.blue*255);
    for(var i = 0; i < ledBuffer.getLength(); i++){
      ledBuffer.setRGB(i, red, green, blue);
    }
  }

  /**
   * @param color
   * @param brightness between 0 and 100
   */
  public void setColor(Color color, int brightness) {
    var hsv = new HSVColor(color);
    hsv.value = (int) (hsv.value*brightness/100.0);
    for(var i = 0; i < ledBuffer.getLength(); i++){
      ledBuffer.setHSV(i, hsv.hue, hsv.saturation, hsv.value);
    }
  }

  private void setColorManual(int red, int green, int blue) {
    for(var i = 0; i < ledBuffer.getLength(); i++){
      ledBuffer.setRGB(i, red, green, blue);
    }
  }

  public void preparing() {
    setColor(Color.kOrange);
    blinkin1.setPulseTimeMicroseconds(BlinkenPattern.solidOrange.us());
    blinkin2.setPulseTimeMicroseconds(BlinkenPattern.solidOrange.us());
  }

  public void ready() {
    setColor(Color.kGreen);
    blinkin1.setPulseTimeMicroseconds(BlinkenPattern.solidGreen.us());
    blinkin2.setPulseTimeMicroseconds(BlinkenPattern.solidGreen.us());
  }

  public Command showTeamColor() {
    return new RunCommand(()->{
      var color = DriverStation.getAlliance();
      if (color.isPresent()) {
        if (color.get() == DriverStation.Alliance.Red) {
          this.setColor(Color.kRed, this.matchBrightnessScaling(10, 100));
          blinkin1.setPulseTimeMicroseconds(BlinkenPattern.solidRed.us());
          blinkin2.setPulseTimeMicroseconds(BlinkenPattern.solidRed.us());
        }
        if (color.get() == DriverStation.Alliance.Blue) {
          this.setColor(Color.kBlue, this.matchBrightnessScaling(10, 100));
          blinkin1.setPulseTimeMicroseconds(BlinkenPattern.solidBlue.us());
          blinkin2.setPulseTimeMicroseconds(BlinkenPattern.solidBlue.us());
        }
        return;
      }
      this.setColor(Color.kPurple, 10);
      blinkin1.setPulseTimeMicroseconds(BlinkenPattern.solidViolet.us());
      blinkin2.setPulseTimeMicroseconds(BlinkenPattern.solidViolet.us());
    },this)
    .ignoringDisable(true)
    ;
  }

  public Command set5vLedStrip(){
    return new InstantCommand(()->{
      this.blinkin1.setPulseTimeMicroseconds(2125);
      this.blinkin2.setPulseTimeMicroseconds(2125);
    });
  }
  public Command showNoteIntake() {
    return new RunCommand(
      ()->{this.setColor(Color.kOrangeRed);
      this.blinkin1.setPulseTimeMicroseconds(BlinkenPattern.solidRedOrange.us());
      this.blinkin2.setPulseTimeMicroseconds(BlinkenPattern.solidRedOrange.us());
    }, this)
    .withTimeout(2);
  }

    
  public Command readyLights(BooleanSupplier ...  conditions) {
    return new RunCommand(()->{
      for (BooleanSupplier condition : conditions) {
        if (!condition.getAsBoolean()) {
          preparing();
          return;
        }
      }
      ready();
    }, this);
  } 


}
