// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.RangingMode;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Passthrough extends SubsystemBase {
  //Define SparkMax
  public CANSparkMax motor = new CANSparkMax(14, MotorType.kBrushless); //
  //Define motor speed, adjust

  double kPassthroughSpeed = 0.1;
  //LaserCAN Sensor Setup
  public LaserCan lasercan = new LaserCan(20);

  /** where we want the game piece under ideal conditions, in mm */
  // public final Measure<Distance> kIdealDistance = Units.Millimeters.of(800);
  public final double kIdealDistance = 800.0;
  /** distance where we're confident game piece is loaded, and loading can stop. In mm */
  public final double kBlockedDistance = 100.0;
  /** distance to the far side of passthrough when unobstructed, in mm */
  public final double kFarWallDistance = 400.0; //mm
  //LaserCan Measurements


  /** Creates a new Passthrough. */
  public Passthrough() {
    motor.restoreFactoryDefaults();
    motor.clearFaults();
    //Safety inplace
    motor.setSmartCurrentLimit(30);

    motor.getPIDController().setP(0.1);

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
  // 
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
  }
}
