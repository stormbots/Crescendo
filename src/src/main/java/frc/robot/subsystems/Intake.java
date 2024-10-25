// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import java.util.Optional;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.RangingMode;
import au.grapplerobotics.LaserCan.TimingBudget;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  //Define SparkMax
  private CANSparkMax motor = new CANSparkMax(9, MotorType.kBrushless);
  //Define motor speed, adjust
  private double kIntakeSpeed = 1.0;
  private LaserCan lasercan = new LaserCan(21);
  /** distance to the far side of passthrough when unobstructed, in mm */
  private final double kFarWallDistance = 355.0; //mm
  /** distance where we're confident game piece is loaded, and loading can stop. In mm */
  private final double kBlockedDistance = 185.0; //mm


  /** Creates a new Intake. */
  public Intake() {
    motor.restoreFactoryDefaults();
    motor.clearFaults();
    //Safety
    motor.setSmartCurrentLimit(60);
    motor.setIdleMode(IdleMode.kBrake);

    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 200);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 200);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 1000);
    motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);

    motor.burnFlash();
    
    try {
      lasercan.setRangingMode(RangingMode.SHORT);
      lasercan.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS);
      SmartDashboard.putBoolean("intake/LaserConfig'd", true);
    } catch (ConfigurationFailedException e) {
      SmartDashboard.putBoolean("intake/LaserConfig'd", false);
    }
  }

  public Optional<Measure<Distance>> getSensorReading(){
    var reading = lasercan.getMeasurement();    
    if(reading == null){return Optional.empty();}
    return Optional.of(Units.Millimeters.of(reading.distance_mm));
  }

  public Measure<Distance> getSensorDistance() {   
    var measurement = getSensorReading();
    var distance = measurement.orElseGet(()->Units.Millimeters.of(kFarWallDistance));
    return distance;
  }

  public boolean isBlocked() {
    var distance = getSensorDistance();
    return distance.in(Units.Millimeters) < kBlockedDistance;
  }

  public void setPower(double speed) {
    motor.set(speed);
  }

  //Intake On
  public void intake(){
    motor.set(kIntakeSpeed);
  }
  //Intake Off
  public void stop(){
    motor.set(0.0);
    PowerManager.getInstance().setPowerDraw(0, this);
  }
  //Intake Eject (use when object stuck)
  public void eject(){
    motor.set(-kIntakeSpeed);
  }

  public double getVelocity() {
    return motor.getEncoder().getVelocity();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("currenttesting/intake", motor.getOutputCurrent());
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("seenote/intakelc", isBlocked());
    SmartDashboard.putNumber("intake/sensordistance", getSensorDistance().in(Units.Millimeters));
    SmartDashboard.putNumber("intake/velocity", motor.getEncoder().getVelocity());
  }
}
