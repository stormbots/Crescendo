// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Lighting;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Leds;

  
public class LightingProgressBar extends Command {
  /** Creates a new LightingColor. */
  Leds leds;
  Color backGroundColor;
  Color progressColor;
  double timeLimt;
  double startTime;
  boolean finished;
  double percentOutput;
  int value;


  public LightingProgressBar(Leds leds, Color backGroundColor, Color progressColor, double timeLimt, double percentOutput) {
    this.leds = leds;
    this.backGroundColor = backGroundColor;
    this.progressColor = progressColor;
    this.timeLimt = timeLimt;
    this.percentOutput = percentOutput;
    value = (int)Math.round((255*(percentOutput/100)));

    addRequirements(leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    finished = false;
    // leds.ledStrip.setData(leds.ledBuffer);
    // leds.ledStrip.start();
    startTime = leds.getTime();
    leds.setColor(backGroundColor);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    var currentTime = leds.getTime();
    var elapsedTime = currentTime-startTime;
    double timePerLED = (timeLimt / leds.ledBuffer.getLength());
    var channels = (double)(Math.round((elapsedTime / timePerLED)));
    if (elapsedTime>timeLimt+.5){
      finished = true;
    }
    leds.setColor(progressColor, value);
    // if (channels + 1 > leds.ledBuffer.getLength()){
    //   finished = true;
    // }
    //do any time math
    //set color

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leds.setColor(backGroundColor);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }

}
