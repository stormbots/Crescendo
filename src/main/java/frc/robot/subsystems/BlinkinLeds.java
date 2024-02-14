// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BlinkinLeds extends SubsystemBase {
  Spark blinkin;
  double pattern;
  /** Creates a new blinkinLeds. */
  public BlinkinLeds() {
    blinkin = new Spark(8);
  }
  // SmartDashboard.putNumber("blinkin/pattern", pattern);
  public void setBlinkinManual(double pattern){
    // this.pattern = pattern;
    blinkin.set(pattern);
  }

  @Override
  public void periodic() {
    pattern = SmartDashboard.getNumber("blinkin/pattern", .99);
    setBlinkinManual(pattern);
    // This method will be called once per scheduler run
  }

  public Command blinkinTeamColor(){
    return new RunCommand(()->{
      var color = DriverStation.getAlliance();
      if (color.isPresent()){
        if (color.get() == DriverStation.Alliance.Red){
          this.setBlinkinManual(.61);
        }
        if (color.get() == DriverStation.Alliance.Blue){
          this.setBlinkinManual(.87);
        }
        return;
      }
      this.setBlinkinManual(.91);
    },this)
    .ignoringDisable(true)
    ;
  }

  public Command blinkinNoteIntake(){
    return new RunCommand(()->this.setBlinkinManual(.63),this).withTimeout(2);
  }

}
