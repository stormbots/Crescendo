// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.stormbots.Clamp;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public CANSparkMax shooterMotor = new CANSparkMax(13, MotorType.kBrushless);
  private SparkPIDController pidController = shooterMotor.getPIDController();
  private SparkAbsoluteEncoder  shooterAbsEncoder = shooterMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
  private double shooterSetPoint = 0.0;

  public Shooter() {
    shooterMotor.restoreFactoryDefaults();

    //TODO: REMOVE THIS
    shooterMotor.getEncoder().setPosition(0);

    pidController = shooterMotor.getPIDController();
    // pidController.setFeedbackDevice(shooterAbsEncoder);

    // Measure<Angle> deg = Units.Degrees.of(360);
    // shooterAbsEncoder.setPositionConversionFactor(360.0);
    // shooterAbsEncoder.setVelocityConversionFactor(shooterAbsEncoder.getPositionConversionFactor()/60.0);
    shooterMotor.getEncoder().setPositionConversionFactor(360);
    shooterMotor.getEncoder().setVelocityConversionFactor(shooterMotor.getEncoder().getPositionConversionFactor()/60.0);
    // shooterMotor.getEncoder().setPosition(shooterAbsEncoder.getPosition()); //when syncing encoders at start, if abs is below 0 (360-350 and such), the relative encoder will sync and not move due to soft limits
    
    shooterMotor.setSoftLimit(SoftLimitDirection.kReverse, 0);
    shooterMotor.setSoftLimit(SoftLimitDirection.kForward,180);

    shooterMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    shooterMotor.enableSoftLimit(SoftLimitDirection.kForward, true);

    shooterMotor.setSmartCurrentLimit(30);

    //closed-loop control
    pidController.setP(0.0001);

    //current limits?
    //soft limits

    shooterMotor.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {

    // shooterMotor.set(0.1);
    // shooterMotor.getPIDController().setReference(0, com.revrobotics.CANSparkBase.ControlType.kPosition);

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("shooter/rotations", shooterMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("shooter/output", shooterMotor.getAppliedOutput());
    SmartDashboard.putNumber("shooter/absEncoder", getShooterAngleAbsolute());
    SmartDashboard.putNumber("shooter/encoder", shooterMotor.getEncoder().getPosition());
  }

  public void moveShooter(double speed) {
    shooterMotor.set(speed);
  }

  public double getShooterAngle() {
    return shooterMotor.getEncoder().getPosition();
  }

  public double getShooterAngleAbsolute() {
    return shooterAbsEncoder.getPosition(); //in rotations, need to do limit
  }

  public double isOnTarget(){
    //TODO figure out better tolerances that make sense
    return Clamp.clamp(shooterAbsEncoder.getPosition(), shooterSetPoint-3, shooterSetPoint+3);
  }

  public void setAngle(double degrees) {
    this.shooterSetPoint = degrees;
    var shooterFF = Math.cos(Math.toRadians(getShooterAngle()));
    pidController.setReference(degrees, com.revrobotics.CANSparkBase.ControlType.kPosition, 0, shooterFF,ArbFFUnits.kPercentOut); //TODO: voltage control
  }

  public Command getDebugSetAngle(double degrees) {
    return new RunCommand(()->setAngle(degrees), this);
  }  

  public Command getManualMoveCommand(double speed){
    return new RunCommand(()->{moveShooter(speed);}, this)
    .finallyDo((end)->moveShooter(0));
  }

  public TrapezoidProfile.State getState(){
    // return new TrapezoidProfile.State(shooterAbsEncoder.getPosition(), shooterAbsEncoder.getVelocity());
    return new TrapezoidProfile.State(shooterMotor.getEncoder().getPosition(), shooterMotor.getEncoder().getVelocity());
  }
}
