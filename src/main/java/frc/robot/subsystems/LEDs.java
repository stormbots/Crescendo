package frc.robot.subsystems;

import com.stormbots.BlinkenPattern;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Leds extends SubsystemBase {
  public AddressableLED ledStrip;
  public AddressableLEDBuffer ledBuffer;
  boolean hasRun;
  Spark blinkin = new Spark(8);

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
    hasRun = false;

    // blinkin.setBoundsMicroseconds(2000, 1501, 1500, 1499, 1000);
    
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

  public int matchBrightnessScaling(int disabledBrightness, int enabledBrightness){
    if (DriverStation.isDisabled()){
      return disabledBrightness;
    }
    return enabledBrightness;
  }

  public void setColor(Color color){
    int red = (int)(color.red);
    int green = (int)(color.green);
    int blue = (int)(color.blue);
    for(var i = 0; i < ledBuffer.getLength(); i++){
      ledBuffer.setRGB(i, red, green, blue);
    }
  }
  /**
   * @param color
   * @param brightness between 0 and 100
   */
  public void setColor(Color color, int brightness){
    var hsv = new HSVColor(color);
    hsv.value = (int) (hsv.value*brightness/100.0);
    for(var i = 0; i < ledBuffer.getLength(); i++){
      ledBuffer.setHSV(i, hsv.hue, hsv.saturation, hsv.value);
    }
  }

  private void setColorManual(int red, int green, int blue){
    for(var i = 0; i < ledBuffer.getLength(); i++){
      ledBuffer.setRGB(i, red, green, blue);
    }
  }


    public Command showTeamColor(){
      return new RunCommand(()->{
        var color = DriverStation.getAlliance();
        if (color.isPresent()){
          if (color.get() == DriverStation.Alliance.Red){
            this.setColor(Color.kRed, this.matchBrightnessScaling(10, 100));
            blinkin.set(BlinkenPattern.solidRed.pwm());
          }
          if (color.get() == DriverStation.Alliance.Blue){
            this.setColor(Color.kBlue, this.matchBrightnessScaling(10, 100));
            blinkin.set(BlinkenPattern.solidBlue.pwm());
          }
          return;
        }
        this.setColor(Color.kPurple, 10);
        blinkin.set(BlinkenPattern.solidViolet.pwm());
      },this)
      .ignoringDisable(true)
      ;
    }

    public Command showNoteIntake(){
      return new RunCommand(()->{this.setColor(Color.kOrangeRed);
      this.blinkin.set(BlinkenPattern.solidRedOrange.pwm());},this).withTimeout(2);
    }
    
  }

