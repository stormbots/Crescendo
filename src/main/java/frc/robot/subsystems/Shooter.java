// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.stormbots.Lerp;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public CANSparkMax shooterMotor = new CANSparkMax(14, MotorType.kBrushless);
  private SparkPIDController pidController = shooterMotor.getPIDController();
  private DutyCycleEncoder shooterAbsEncoder = new DutyCycleEncoder(21);
  private double shooterSetPoint = 0.0;
  private Lerp shooterAnalogLerp = new Lerp(0, 0, 0, 0);

  public Shooter() {
    shooterMotor.restoreFactoryDefaults();

    pidController = shooterMotor.getPIDController();

    //closed-loop control
    double kP = 0; //1.0 works on testbench
    double kI = 0;
    double kD = 0;
    double kIz = 0;

    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(0);

    //current limits?
    //soft limits

    shooterMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {

    // shooterMotor.set(0.1);
    // shooterMotor.getPIDController().setReference(0, com.revrobotics.CANSparkBase.ControlType.kPosition);

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Rotations", shooterMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("output", shooterMotor.getAppliedOutput());
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
    pidController.setReference(setPoint, com.revrobotics.CANSparkBase.ControlType.kPosition, 0, shooterFF,ArbFFUnits.kPercentOut); //TODO: voltage control
  }

  public Command getManualMoveCommand(double speed){
    return new RunCommand(()->{moveShooter(speed);}, this)
    .finallyDo((end)->moveShooter(0));
  }
}
