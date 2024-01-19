// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  //Define SparkMax
  public CANSparkMax motor = new CANSparkMax(30, MotorType.kBrushless);
  //Define motor speed, adjust
  double kIntakeSpeed = 0.1;

  /** Creates a new Intake. */
  public Intake() {
    //Safety
    motor.setSmartCurrentLimit(30);
  }


  //Intake On
  public void intakeOn(){
    motor.set(kIntakeSpeed);
  }
  //Intake Off
  public void intakeOff(){
    motor.set(0.0);
  }
  //Intake Eject (use when object stuck)
  public void intakeEject(){
    motor.set(-kIntakeSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
