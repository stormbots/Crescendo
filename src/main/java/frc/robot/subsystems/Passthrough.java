// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Passthrough extends SubsystemBase {
  //Define SparkMax
  public CANSparkMax passthroughMotor = new CANSparkMax(14, MotorType.kBrushless);;
  //Define motor speed, adjust

  double kPassthroughSpeed = 0.1;
  //LaserCAN Sensor Setup
  public LaserCan passthroughSensor = new LaserCan(20);

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
    //Safety inplace
    passthroughMotor.setSmartCurrentLimit(30);

  }

  public Optional<Measure<Distance>> getSensorReading(){
    var reading = passthroughSensor.getMeasurement();
    // SmartDashboard.putNumber("passthrough/rawmeasurement", reading.distance_mm);
    
    if(reading == null){return Optional.empty();}
    return Optional.of(Units.Millimeters.of(reading.distance_mm));
  }

  //This for manual option for driver
  //Passthrough On
  public void intake(){
    if(isBlocked() == false) {
      passthroughMotor.set(kPassthroughSpeed);
    } else {
      //This where we will do passthroughAlignNote
      // passthroughMotor.set(0.0);
    }

  }
  //Passthrough Off
  public void stop() {
    passthroughMotor.set(0.0);
  }
  //Passthrough Out
  public void eject() {
    passthroughMotor.set(-kPassthroughSpeed);
  }
  // 
  public Measure<Distance> sensorValues() {   
    var measurement = getSensorReading(); 
    var distance = measurement.orElseGet(()->Units.Millimeters.of(kFarWallDistance));
    return distance;
  }

  //This will be area for setup for sensor
  public boolean isBlocked() {
    var distance = sensorValues();
    

    return distance.in(Units.Millimeters) < kBlockedDistance;
  }
  //PID setup for passthough
  public void setPassthroughPID(double p, double i, double d){
    passthroughMotor.getPIDController().setP(p);
    passthroughMotor.getPIDController().setI(i);
    passthroughMotor.getPIDController().setD(d);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("passthrough/isBlocked", isBlocked());
    SmartDashboard.putNumber("passthrough/value", sensorValues().in(Units.Millimeters));
  }
}
