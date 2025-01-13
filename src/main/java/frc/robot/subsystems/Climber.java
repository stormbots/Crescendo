// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.stormbots.Clamp;
import com.studica.frc.AHRS;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  private SparkMax leftMotor = new SparkMax(Robot.isCompbot?17:16, MotorType.kBrushless);
  private SparkMax rightMotor = new SparkMax(Robot.isCompbot?18:17, MotorType.kBrushless);
  
  public boolean isHomed=false;
  public final double kHomeCurrentThreshold=5;
  public final double kClimbingCurrentThreshold=10;
  public final double kHomePower=-0.1;
  public final  Distance kMaxHeight=Units.Inches.of(23);;
  public final  Distance kClimbReadyPosition=Units.Inches.of(23.25-6);
  private double positionSetpoint=0;

  public float forwardSoftLimit = (float)(kMaxHeight.in(Units.Inches)-0.2);
  public float climbingReverseSoftLimit = (float)0.1;
  public float defaultReverseSoftLimit = (float)11.5;

  public Climber(AHRS navx) {
    //TODO Auto-generated constructor stub

    var leftconfig = new SparkMaxConfig()
    .smartCurrentLimit(8)
    .apply(Constants.kTypical)
    .idleMode(IdleMode.kCoast)
    ;
    leftconfig.softLimit
    .reverseSoftLimitEnabled(false)
    .reverseSoftLimit(climbingReverseSoftLimit)
    .forwardSoftLimit(forwardSoftLimit)
    .forwardSoftLimitEnabled(true)
    ;
    leftconfig.closedLoop.p(0.3)
    ;
    //duplicate the config in a safe manner
    var rightconfig = new SparkMaxConfig().apply(leftconfig);

    //NOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
    leftconfig.encoder.positionConversionFactor(kMaxHeight.in(Units.Inches)/80.146);//kMaxHeight.in(Units.Inches)/71.69
    rightconfig.encoder.positionConversionFactor(kMaxHeight.in(Units.Inches)/80.146*23.1/25.3);//kMaxHeight.in(Units.Inches)/71.69

    //set soft limits
    leftconfig.inverted(true);
    rightconfig.inverted(true);

    leftMotor.configure(leftconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.configure(rightconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // SmartDashboard.putData("climber/home", new ClimberGoHome(this));

    // var up = Units.Inches.of(20);
    // SmartDashboard.putData("climber/up", new RunCommand(()->setPosition(up),this));
    // var down = Units.Inches.of(0);
    // SmartDashboard.putData("climber/down", new RunCommand(()->setPosition(down),this));
  }


  public void setPower(double power){
    leftMotor.set(power);
    rightMotor.set(power);
  }

  SparkBaseConfig confIdleModeBrake = new SparkMaxConfig().idleMode(IdleMode.kBrake);
  SparkBaseConfig confIdleModeCoast = new SparkMaxConfig().idleMode(IdleMode.kCoast);
  public void setIdleMode(IdleMode mode){
    switch(mode){
      case kBrake:
      leftMotor.configureAsync(confIdleModeBrake, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      leftMotor.configureAsync(confIdleModeBrake, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      break;
      case kCoast:
      leftMotor.configureAsync(confIdleModeCoast, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
      leftMotor.configureAsync(confIdleModeCoast, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
  }

  public Boolean isAtHomePosition(){
    return (leftMotor.getOutputCurrent()>kHomeCurrentThreshold && rightMotor.getOutputCurrent()>kHomeCurrentThreshold);
  }

  public void setHomed(){
    isHomed = true;

    var config=new SparkMaxConfig()
    .smartCurrentLimit(30)
    .idleMode(IdleMode.kBrake);
    config.softLimit
    .reverseSoftLimitEnabled(true)
    .reverseSoftLimit(defaultReverseSoftLimit);
    

    for(SparkMax motor : new SparkMax[]{leftMotor,rightMotor} ){
      motor.getEncoder().setPosition(0);
      motor.configureAsync(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

  }

  public void setPosition(Distance target){
    setPosition(target.in(Units.Inches));
  }

  /** Takes target in inches */
  private void setPosition(double target){
    this.positionSetpoint = target;
    var ff = 0.0;

    if(leftMotor.getOutputCurrent() > kClimbingCurrentThreshold &&
      rightMotor.getOutputCurrent() > kClimbingCurrentThreshold
    ){
      ff = -0.1;
    }

    leftMotor.getClosedLoopController().setReference(
      target, ControlType.kPosition, ClosedLoopSlot.kSlot0,
      ff,ArbFFUnits.kPercentOut);
    rightMotor.getClosedLoopController().setReference(
      target, ControlType.kPosition, ClosedLoopSlot.kSlot0,
      ff,ArbFFUnits.kPercentOut);
  }

  public Boolean isAtSetpoint(){
    var leftPosOk = Clamp.bounded(leftMotor.getEncoder().getPosition(), positionSetpoint-0.5, positionSetpoint+0.5); 
    var rightPosOk = Clamp.bounded(rightMotor.getEncoder().getPosition(), positionSetpoint-0.5, positionSetpoint+0.5); 
    if(leftPosOk==false || rightPosOk ==false ) return false; 

    return true;
  }

  public Distance getPosition(){
    var average = (leftMotor.getEncoder().getPosition() + rightMotor.getEncoder().getPosition())/2;
    return Units.Inches.of(average);
  }

  public void setReverseSoftLimit(double limit){
    var config = new SparkMaxConfig();
    config.softLimit
      .reverseSoftLimit(limit)
      .reverseSoftLimitEnabled(true)
    ;

    leftMotor.configureAsync(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    rightMotor.configureAsync(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("currenttesting/leftClimber", leftMotor.getOutputCurrent());
    SmartDashboard.putNumber("currenttesting/rightClimber", rightMotor.getOutputCurrent());
    // SmartDashboard.putNumber("/climber/rightCurrent", rightMotor.getOutputCurrent());
    SmartDashboard.putNumber("/climber/leftPosition", leftMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("/climber/rightPosition", rightMotor.getEncoder().getPosition());
    SmartDashboard.putBoolean("/climber/isHomed", isHomed);
    // SmartDashboard.putNumber("/climber/out", leftMotor.getAppliedOutput());

    // double max =  kMaxHeight.in(Units.Inches) - 2.0;
    // double min =  2.0; //temp value

    // if(isHomed && Clamp.bounded(getPosition().in(Units.Inches), min, max)){
    //   leftMotor.setSmartCurrentLimit(30);
    //   rightMotor.setSmartCurrentLimit(30);
    // } else{
    //   leftMotor.setSmartCurrentLimit(4);
    //   rightMotor.setSmartCurrentLimit(4);
    // })



  }
}
