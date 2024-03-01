// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.RangingMode;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Passthrough extends SubsystemBase {
  //Define SparkMax
  public CANSparkMax motor = new CANSparkMax(Robot.isCompbot?10:10 , MotorType.kBrushless); //
  public CANSparkMax motorB = new CANSparkMax(Robot.isCompbot?11:22, MotorType.kBrushless); //
  //Define motor speed, adjust
  private double kPassthroughSpeed=1.0;
  //LaserCAN Sensor Setup
  public LaserCan lasercan = new LaserCan(20);//TODO: Update LaserCAN PB

  /** where we want the game piece under ideal conditions, in mm */
  public final Measure<Distance> kIdealDistance = Units.Millimeters.of(23);
  // public final double kIdealDistance = 23.0;
  /** distance where we're confident game piece is loaded, and loading can stop. In mm */
  public final double kBlockedDistance = 185.0;
  /** distance to the far side of passthrough when unobstructed, in mm */
  public final double kFarWallDistance = 355.0; //mm
  

  /** Creates a new Passthrough. */
  public Passthrough() {
    motor.restoreFactoryDefaults();
    motor.clearFaults();
    motorB.restoreFactoryDefaults();
    motorB.clearFaults();

    motorB.follow(motor,true);//TODO: Check invert
    motor.setInverted(false);

    motor.setIdleMode(IdleMode.kBrake);
    motorB.setIdleMode(IdleMode.kBrake);

    //Safety inplace
    motor.setSmartCurrentLimit(15);

    motor.getPIDController().setP(0.1);
    motor.setClosedLoopRampRate(0.05);

    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 200);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 1000);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);
    
    motorB.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);
    motorB.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 200);
    motorB.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    motorB.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
    motorB.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 1000);
    motorB.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);

    motor.burnFlash();
    motorB.burnFlash();
    try {
      lasercan.setRangingMode(RangingMode.SHORT);
      SmartDashboard.putBoolean("passthrough/LaserConfig'd", true);
    } catch (ConfigurationFailedException e) {
      SmartDashboard.putBoolean("passthrough/LaserConfig'd", false);
    }
  }

  public Optional<Measure<Distance>> getSensorReading(){
    var reading = lasercan.getMeasurement();    
    if(reading == null){return Optional.empty();}
    return Optional.of(Units.Millimeters.of(reading.distance_mm));
  }

  public void setPower(double power){
    motor.set(power);
  }

  //This for manual option for driver
  public void intake(){
    motor.set(kPassthroughSpeed);
  }

  public void stop() {
    motor.set(0.0);
  }

  public void eject() {
    motor.set(-kPassthroughSpeed);
  }
  
  public Measure<Distance> getSensorDistance() {   
    var measurement = getSensorReading(); 
    var distance = measurement.orElseGet(()->Units.Millimeters.of(kFarWallDistance));
    return distance;
  }

  //This will be area for setup for sensor
  public boolean isBlocked() {
    var distance = getSensorDistance();
    return distance.in(Units.Millimeters) < kBlockedDistance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("passthrough/isBlocked", isBlocked());
    SmartDashboard.putNumber("passthrough/value", getSensorDistance().in(Units.Millimeters));
    // SmartDashboard.putNumber("passthrough/outputCurrent", motor.getOutputCurrent());
  }
}
