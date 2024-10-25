package frc.robot.commands.Lighting;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LightEmittingDiodes;
import frc.robot.subsystems.LightEmittingDiodes.HSVColor;

  
public class LightingProgressBarSnap extends Command {
  /** Creates a new LightingColor. */
  LightEmittingDiodes leds;
  Color backGroundColor;
  LightEmittingDiodes.HSVColor progressColor;
  double timeLimit;
  double startTime;
  boolean finished;
  int brightness;
  


  public LightingProgressBarSnap (LightEmittingDiodes leds, Color backGroundColor, Color progressColor, double timeLimit, double brightness) {
    this.leds = leds;
    this.backGroundColor = backGroundColor;
    this.progressColor = leds.new HSVColor(progressColor);
    this.timeLimit = timeLimit;
    this.brightness = (int)Math.round(brightness);
    addRequirements(leds);
  }
    // Use addRequirements() here to declare subsystem dependencies.
    

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    finished = false;
    startTime =leds.getTime();
  leds.setColor(backGroundColor,brightness);
    // SmartDashboard.putNumber(leds/startTime", startTime);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    progressColor.value = (int) (progressColor.value*brightness/100.0);
    var currentTime = leds.getTime();
    var elapsedTime = currentTime-startTime;
    double timePerLED = (timeLimit /leds.ledBuffer.getLength());
    var numberOfChannelsOn = (int)((elapsedTime / timePerLED));
    if (numberOfChannelsOn >leds.ledBuffer.getLength()){
      numberOfChannelsOn =leds.ledBuffer.getLength();
    }
    if (elapsedTime>timeLimit+.5){
      finished =true;
    }
    for(var i = 0; i < numberOfChannelsOn; i++){
    leds.ledBuffer.setHSV(i, progressColor.hue, progressColor.saturation, progressColor.value);
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
