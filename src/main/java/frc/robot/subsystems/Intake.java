// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  //Define SparkMax
  private CANSparkMax motor = new CANSparkMax(9, MotorType.kBrushless);
  //Define motor speed, adjust
  private double kIntakeSpeed = 1.0;

  /** Creates a new Intake. */
  public Intake() {
    motor.restoreFactoryDefaults();
    motor.clearFaults();
    //Safety
    motor.setSmartCurrentLimit(60);

    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 200);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 200);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 1000);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);

    motor.burnFlash();
  }
  
  public void setPower(double speed) {
    motor.set(speed);
  }


  //Intake On
  public void intake(){
    motor.set(kIntakeSpeed);
  }
  //Intake Off
  public void stop(){
    motor.set(0.0);
    PowerManager.getInstance().setPowerDraw(0, this);
  }
  //Intake Eject (use when object stuck)
  public void eject(){
    motor.set(-kIntakeSpeed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("currenttesting/intake", motor.getOutputCurrent());
    // This method will be called once per scheduler run
  }
}
