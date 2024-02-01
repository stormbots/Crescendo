// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ClimberGoHome;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  CANSparkFlex leftMotor = new CANSparkFlex(21, MotorType.kBrushless);
  CANSparkFlex rightMotor = new CANSparkFlex(22, MotorType.kBrushless);
  public boolean isHomed=false;
  public final double kHomeCurrentThreshold=1;
  public final double kClimbingCurrentThreshold=10;
  public final double kHomePower=-0.05;
  public final  Measure<Distance> kMaxHeight=Units.Inches.of(24);;
  public final  Measure<Distance> kClimbReadyPosition=Units.Inches.of(24);

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
      leftMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
      leftMotor.getEncoder().setPosition(0);
      rightMotor.setSmartCurrentLimit(30);
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
