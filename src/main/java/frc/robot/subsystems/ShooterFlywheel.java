// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class ShooterFlywheel extends SubsystemBase {
  public CANSparkMax leftFlywheel = new CANSparkMax(23, MotorType.kBrushless);
  public CANSparkMax rightFlywheel1 = new CANSparkMax(24, MotorType.kBrushless);
  public CANSparkMax rightFlywheel2 = new CANSparkMax(25, MotorType.kBrushless);
  public RelativeEncoder leftFlywheelEncoder;
  public RelativeEncoder rigtFlywheelEncoder;
  public SparkPIDController leftFlywheelPIDController;
  public SparkPIDController rightFlywheelPIDController;
  double targetRPM = 0;
   
  /** Creates a new Flywheel. */
  public ShooterFlywheel() {
    
    //Set up components
    rightFlywheel2.follow(rightFlywheel1);
    leftFlywheel.setInverted(true);
    leftFlywheelEncoder = leftFlywheel.getEncoder();
    rigtFlywheelEncoder = rightFlywheel1.getEncoder();
    leftFlywheelPIDController = leftFlywheel.getPIDController();
    rightFlywheelPIDController = rightFlywheel1.getPIDController();
    int SmartMotionSlotID = 0; 
     
    //Set Left PID
    leftFlywheelPIDController.setP(0);
    leftFlywheelPIDController.setI(0);                                                             
    leftFlywheelPIDController.setD(0);
    leftFlywheelPIDController.setFF(1/5760.0); //1/200 seemed better?
    leftFlywheelPIDController.setOutputRange(0,1); //dont know if we need this, adding just in case
    
    
    leftFlywheelPIDController.setSmartMotionMaxVelocity(5760.0, SmartMotionSlotID);
    leftFlywheelPIDController.setSmartMotionMaxAccel(5760/1.0, SmartMotionSlotID);
    
    //Set Right PID
    
    rightFlywheelPIDController.setP(0);
    rightFlywheelPIDController.setI(0);
    rightFlywheelPIDController.setD(0);
    rightFlywheelPIDController.setFF(1/5760.0);
    rightFlywheelPIDController.setOutputRange(0, 1); //dont know if we need this, adding just in case

    
    rightFlywheelPIDController.setSmartMotionMaxVelocity(5760.0, SmartMotionSlotID);
    rightFlywheelPIDController.setSmartMotionMaxAccel(5760/1.0, SmartMotionSlotID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("targetRPM", targetRPM);
    SmartDashboard.putBoolean("isOnTarget", isOnTarget(leftFlywheel));
    SmartDashboard.putNumber("currentRPM", getRPM(leftFlywheel));    
  }
  public void setRPM(double targetRPM) {
    this.targetRPM = targetRPM;
    leftFlywheelPIDController.setReference(targetRPM, CANSparkMax.ControlType.kSmartVelocity);
    //rightFlywheelPIDController.setReference(targetRPM, CANSparkMax.ControlType.kSmartVelocity);
  }

  public boolean isOnTarget(CANSparkMax motor){
    RelativeEncoder motorEncoder = motor.getEncoder();
    double currentRPM = motorEncoder.getVelocity();
    double setRPM = targetRPM;
    double error = Math.abs(setRPM-currentRPM);
    double tolerance = setRPM * .05;

    if (error > tolerance) return false;
    return true;
  }
  public double getRPM(CANSparkMax motor){
    RelativeEncoder motorEncoder = motor.getEncoder();
    double currentRPM = motorEncoder.getVelocity();
    return currentRPM;
  }

}
