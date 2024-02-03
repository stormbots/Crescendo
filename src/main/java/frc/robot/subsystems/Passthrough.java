// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Passthrough extends SubsystemBase {
  //Define SparkMax
  public CANSparkMax motor = new CANSparkMax(10, MotorType.kBrushless);;
  //Define motor speed, adjust
  double kPassthroughSpeed = 0.1;
  //Additional Sensor (unknown currently ;( )


  /** Creates a new Passthrough. */
  public Passthrough() {
    //Safety inplace
    motor.setSmartCurrentLimit(30);
  }

  //This for manual option for driver
  //Passthrough On
  public void passthoughOn(){
    motor.set(kPassthroughSpeed);
  }
  //Passthrough Off
  public void passthoughOff() {
    motor.set(0.0);
  }
  //Passthrough Out
  public void passthroughOut() {
    motor.set(-kPassthroughSpeed);
  }
  //This will be area for setup for sensor
  public void sensorStop() {
    //if someSensor == no note/orange not dected
    // passthroughOn
    //if someSensor == note/orange dected
    // passthroughOff
     
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
