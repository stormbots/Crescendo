// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
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
    shooterMotor.clearFaults();
    shooterMotor.restoreFactoryDefaults();

    shooterMotor.setClosedLoopRampRate(0.05);

    // pidController.setFeedbackDevice(shooterAbsEncoder);  //WARNING: This is potentially unsafe due to controlling through the discontinuity at 0; do not use. 
    pidController.setFeedbackDevice(shooterMotor.getEncoder()); //Make sure we revert to native encoder for PID

    //Configure Absolute encoder to accurate values
    shooterAbsEncoder.setPositionConversionFactor(360.0);
    shooterAbsEncoder.setInverted(false);
    shooterAbsEncoder.setVelocityConversionFactor(shooterAbsEncoder.getPositionConversionFactor()); //native unit is RPS

    //Configure relative encoder
    shooterMotor.getEncoder().setPositionConversionFactor(56.8/15.1);
    shooterMotor.getEncoder().setVelocityConversionFactor(shooterMotor.getEncoder().getPositionConversionFactor()/60.0); //Native unit is RPM, so convert to RPS
    syncEncoders();

    shooterMotor.setSoftLimit(SoftLimitDirection.kReverse, 5);
    shooterMotor.setSoftLimit(SoftLimitDirection.kForward,45);
    shooterMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    shooterMotor.enableSoftLimit(SoftLimitDirection.kForward, true);

    shooterMotor.setSmartCurrentLimit(20);

    //closed-loop control
    pidController.setP(6.0/1.2/360.0);

    shooterMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("shooter/rotations", shooterMotor.getEncoder().getPosition());
    // SmartDashboard.putNumber("shooter/output", shooterMotor.getAppliedOutput());
    // SmartDashboard.putNumber("shooter/absEncoder", getShooterAngleAbsolute());
    // SmartDashboard.putNumber("shooter/encoder", shooterMotor.getEncoder().getPosition());
    // SmartDashboard.putNumber("shooter/outputCurrent", shooterMotor.getOutputCurrent());
    // SmartDashboard.putNumber("shooter/TrapezoidProfile", getState().velocity);
  }

  /** Align the absolute and relative encoders, should the need arise */
  public void syncEncoders(){
    var position = shooterAbsEncoder.getPosition();
    if(position > 225){
      //Account for discontinuity, set relative to negative position
      shooterMotor.getEncoder().setPosition(position-360);
    }else{
      shooterMotor.getEncoder().setPosition(position);
    }
  }

  public void stopShooter(){
    shooterMotor.set(0);
  }
  
  public void moveShooter(double speed) {
    shooterMotor.set(speed + getShooterFFPercent());
  }

  public double getShooterAngle() {
    return shooterMotor.getEncoder().getPosition();
  }

  public double getShooterAngleAbsolute() {
    return shooterAbsEncoder.getPosition(); //in rotations, need to do limit
  }

  public double isOnTarget(){
    //TODO figure out better tolerances that make sense
    return Clamp.clamp(shooterMotor.getEncoder().getPosition(), shooterSetPoint-3, shooterSetPoint+3);
  }

  public double getShooterFFPercent(){
    var  kCosFFGain = 0.06;
    return kCosFFGain*Math.cos(Math.toRadians(getShooterAngle()));
  }

  public void setAngle(double degrees) {
    this.shooterSetPoint = degrees;
    Clamp.clamp(degrees, shooterMotor.getSoftLimit(SoftLimitDirection.kReverse), shooterMotor.getSoftLimit(SoftLimitDirection.kForward)); 
    pidController.setReference(degrees, ControlType.kPosition, 0, getShooterFFPercent(),ArbFFUnits.kPercentOut);
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
