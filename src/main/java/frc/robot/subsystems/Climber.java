// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.stormbots.Clamp;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  private CANSparkFlex leftMotor = new CANSparkFlex(21, MotorType.kBrushless);
  private CANSparkFlex rightMotor = new CANSparkFlex(22, MotorType.kBrushless);
  public boolean isHomed=false;
  public final double kHomeCurrentThreshold=1;
  public final double kClimbingCurrentThreshold=10;
  public final double kHomePower=-0.05;
  public final  Measure<Distance> kMaxHeight=Units.Inches.of(24);;
  public final  Measure<Distance> kClimbReadyPosition=Units.Inches.of(24);
  private double positionSetpoint=0;

  public Climber(AHRS navx) {
    //TODO Auto-generated constructor stub

    setIdleMode(IdleMode.kCoast);    

    //set soft limits
    for(CANSparkBase motor : new CANSparkBase[]{leftMotor,rightMotor} ){
      motor.setSoftLimit(SoftLimitDirection.kReverse, (float)0.5);
      motor.enableSoftLimit(SoftLimitDirection.kReverse, false);

      motor.setSoftLimit(SoftLimitDirection.kForward, (float)kMaxHeight.in(Units.Inches));
      motor.enableSoftLimit(SoftLimitDirection.kForward, true);

      motor.getEncoder().setPositionConversionFactor(1.0);
      motor.setSmartCurrentLimit(5);
    }
  }


  public void setPower(double power){
    leftMotor.set(power);
    rightMotor.set(power);
  }

  public void setIdleMode(IdleMode mode){
    leftMotor.setIdleMode(mode);
    rightMotor.setIdleMode(mode);
  }

  public Boolean isAtHomePosition(){
    return (leftMotor.getOutputCurrent()>kHomeCurrentThreshold && rightMotor.getOutputCurrent()>kHomeCurrentThreshold);
  }

  public void setHomed(){
    isHomed = true;

    for(CANSparkBase motor : new CANSparkBase[]{leftMotor,rightMotor} ){
      motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
      motor.getEncoder().setPosition(0);
      motor.setSmartCurrentLimit(30);
    }
  }

  public void setPosition(double target){
    this.positionSetpoint = target;
    var ff = 0.0;

    if(leftMotor.getOutputCurrent() > kClimbingCurrentThreshold &&
      rightMotor.getOutputCurrent() > kClimbingCurrentThreshold
    ){
      ff = -0.1;
    }

    leftMotor.getPIDController().setReference(
      target, ControlType.kPosition, 0,
      ff,ArbFFUnits.kPercentOut);
    rightMotor.getPIDController().setReference(
      target, ControlType.kPosition, 0,
      ff,ArbFFUnits.kPercentOut);
  }

  public Boolean isAtSetpoint(){
    var leftPosOk = Clamp.bounded(leftMotor.getEncoder().getPosition(), positionSetpoint-0.5, positionSetpoint+0.5); 
    var rightPosOk = Clamp.bounded(rightMotor.getEncoder().getPosition(), positionSetpoint-0.5, positionSetpoint+0.5); 
    if(leftPosOk==false || rightPosOk ==false ) return false; 

    return true;
  }

  public Measure<Distance> getPosition(){
    var average = (leftMotor.getEncoder().getPosition() + rightMotor.getEncoder().getPosition())/2;
    return Units.Inches.of(average);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("/climber/leftCurrent", leftMotor.getOutputCurrent());
    SmartDashboard.putNumber("/climber/rightCurrent", rightMotor.getOutputCurrent());
  }
}
