// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Lerp;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public CANSparkMax shooterMotor = new CANSparkMax(21, MotorType.kBrushless);
  private SparkPIDController pidController = shooterMotor.getPIDController();
  private DutyCycleEncoder shooterAbsEncoder = new DutyCycleEncoder(21);
  private double shooterSetPoint = 0.0;
  private Lerp shooterAnalogLerp = new Lerp(0, 0, 0, 0);

  //temp values
  private IdleMode idle;
  private CANSparkMax.ControlType controlType;


  public Shooter() {
    shooterMotor.restoreFactoryDefaults();

    pidController = shooterMotor.getPIDController();

    //closed-loop control
    double kP = 0;
    double kI = 0;
    double kD = 0;
    double kIz = 0;
    double kFF = 0;
    double kMaxOutput = 0;
    double kMinOutput = -0.3;

    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);

    //current limits?
    //soft limits

    shooterMotor.setIdleMode(idle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void moveShooter(double speed) {
    shooterMotor.set(speed);
  }

  public double getShooterAngle() {
    return shooterMotor.getEncoder().getPosition();
  }

  public double getShooterAngleAbsolute() {
    return shooterAbsEncoder.get(); //in rotations, need to do limit
  }

  public void setShooterPID(double setPoint) {
    this.shooterSetPoint = setPoint;
    var shooterFF = Lerp.lerp(0, 0, 0, 0, 0);
    shooterFF*=Math.cos(Math.toRadians(getShooterAngle()));
    pidController.setReference(setPoint, controlType, 21, shooterFF); //TODO: voltage control
  }
}
