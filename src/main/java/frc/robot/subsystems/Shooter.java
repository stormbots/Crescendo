// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import java.lang.annotation.Target;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.stormbots.Clamp;
import com.stormbots.LUT;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public CANSparkMax shooterMotor = new CANSparkMax(Robot.isCompbot?14:13, MotorType.kBrushless);
  private SparkPIDController pidController = shooterMotor.getPIDController();
  private SparkAbsoluteEncoder  shooterAbsEncoder = shooterMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
  private double shooterSetPoint = 0.0;

  private double reverseSoftLimit = shooterMotor.getSoftLimit(SoftLimitDirection.kReverse);
  private double forwardSoftLimit = shooterMotor.getSoftLimit(SoftLimitDirection.kForward);

  public static LUT lut = new LUT(new double[][]{
    {54, 42.5, 4000},
    {66, 38.3, 4000},
    {78, 34.1, 4000},
    {90, 30.4, 4000},
    {103, 24.0, 4000},
    {113, 23.6, 4000},
    {124, 20.0, 4500},
    {148, 15.4, 5500}, //farthest shot with dunkarm down
    {173, 13.2, 6000} 
  });

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
    shooterMotor.getEncoder().setPositionConversionFactor(45.0/11.51*0.955);//56.8/15.1
    shooterMotor.getEncoder().setVelocityConversionFactor(shooterMotor.getEncoder().getPositionConversionFactor()/60.0); //Native unit is RPM, so convert to RPS
    syncEncoders();

    shooterMotor.setSoftLimit(SoftLimitDirection.kReverse, 5);
    shooterMotor.setSoftLimit(SoftLimitDirection.kForward,50);
    shooterMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    shooterMotor.enableSoftLimit(SoftLimitDirection.kForward, true);

    shooterMotor.setSmartCurrentLimit(20);

    //closed-loop control
    pidController.setP(6.0/1.2/360.0*1.7*1.1);
    pidController.setI(0.000000003*20);
    pidController.setD(0.00007*50);
    

    shooterMotor.setIdleMode(IdleMode.kCoast);

    
    shooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    shooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
    shooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 1000);
    shooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);

    shooterMotor.burnFlash();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("currenttesting/shooterAngle", shooterMotor.getOutputCurrent());

    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("shooter/rotations", shooterMotor.getEncoder().getPosition());
    // SmartDashboard.putNumber("shooter/output", shooterMotor.getAppliedOutput());
    // SmartDashboard.putNumber("shooter/absEncoder", getShooterAngleAbsolute());
    SmartDashboard.putNumber("shooter/encoder", shooterMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("shooter/target", shooterSetPoint);
    // SmartDashboard.putNumber("shooter/outputCurrent", shooterMotor.getOutputCurrent());
    // SmartDashboard.putNumber("shooter/TrapezoidProfile", getState().velocity);
    // SmartDashboard.putBoolean("shooter/isOnTarget", isOnTarget());
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

  public void setPower(double power){
    shooterMotor.set(power);
  }

  public void stopShooter(){
    shooterMotor.set(0);
    PowerManager.getInstance().setPowerDraw(0, this);
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

  public boolean isOnTarget(){
    var tolerance = 0.5;
    //TODO figure out better tolerances that make sense
    return Clamp.bounded(shooterMotor.getEncoder().getPosition(), shooterSetPoint-tolerance, shooterSetPoint+tolerance);

  }
  public boolean isOnTarget(double position){
    var tolerance = 0.5;
    //TODO figure out better tolerances that make sense
    return Clamp.bounded(position, position-tolerance, position+tolerance);

  }



  public double getShooterFFPercent(){
    var  kCosFFGain = 0.08;
    var ks = 0.0099 *Math.signum(this.shooterSetPoint-shooterMotor.getEncoder().getPosition());
    // 0.08 +- 0.09 according to tests
    return kCosFFGain*Math.cos(Math.toRadians(getShooterAngle())) + ks;
  }

  public void setAngle(double degrees) {
    this.shooterSetPoint = degrees;
    Clamp.clamp(degrees, reverseSoftLimit, forwardSoftLimit); 
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
