// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkFlexFixes;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.ChassisConstants.ModuleConstants;

public class MAXSwerveModule implements Sendable{
  public final CANSparkFlex drivingSparkFlex;
  private final CANSparkMax turningSparkMax;

  public final RelativeEncoder drivingEncoder;
  public final AbsoluteEncoder turningEncoder;

  private final SparkPIDController drivingPIDController;
  private final SparkPIDController turningPIDController;

  private double chassisAngularOffset = 0;
  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  private SimpleMotorFeedforward drivingMotorFeedforward = new SimpleMotorFeedforward(0,0,0);

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset, SimpleMotorFeedforward drivingMotorFeedforward){
    drivingSparkFlex = new CANSparkFlex(drivingCANId, MotorType.kBrushless);
    turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);
    this.drivingMotorFeedforward = drivingMotorFeedforward;

    drivingSparkFlex.clearFaults();
    turningSparkMax.clearFaults();

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    drivingSparkFlex.restoreFactoryDefaults();
    turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    drivingEncoder = drivingSparkFlex.getEncoder();
    turningEncoder = turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    drivingPIDController = drivingSparkFlex.getPIDController();
    turningPIDController = turningSparkMax.getPIDController();
    drivingPIDController.setFeedbackDevice(drivingEncoder);
    turningPIDController.setFeedbackDevice(turningEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    drivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
    drivingEncoder.setVelocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    turningEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    turningPIDController.setPositionPIDWrappingEnabled(true);
    turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    drivingPIDController.setP(ModuleConstants.kDrivingP);
    drivingPIDController.setI(ModuleConstants.kDrivingI);
    drivingPIDController.setD(ModuleConstants.kDrivingD);
    // drivingPIDController.setFF(ModuleConstants.kDrivingFF);  
    drivingPIDController.setOutputRange(ModuleConstants.kDrivingMinOutput,
        ModuleConstants.kDrivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    turningPIDController.setP(ModuleConstants.kTurningP);
    turningPIDController.setI(ModuleConstants.kTurningI);
    turningPIDController.setD(ModuleConstants.kTurningD);
    turningPIDController.setFF(ModuleConstants.kTurningFF);
    turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
        ModuleConstants.kTurningMaxOutput);

    drivingSparkFlex.setIdleMode(IdleMode.kBrake);
    turningSparkMax.setIdleMode(IdleMode.kBrake);
    drivingSparkFlex.setSmartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    drivingEncoder.setAverageDepth(2);
    drivingEncoder.setMeasurementPeriod(8);
    // SparkFlexFixes.setFlexEncoderAverageDepth(drivingSparkFlex, 2);
    // SparkFlexFixes.setFlexEncoderSampleDelta(drivingSparkFlex, 8);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.

    this.chassisAngularOffset = chassisAngularOffset;
    desiredState.angle = new Rotation2d(turningEncoder.getPosition());
    drivingEncoder.setPosition(0);

    drivingSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    drivingSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
    drivingSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 1000);
    drivingSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);
    // drivingSparkFlex.setClosedLoopRampRate(0.02);

    turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    
    // SparkFlexFixes.setFlexEncoderAverageDepth(drivingSparkFlex, 2);
    // SparkFlexFixes.setFlexEncoderSampleDelta(drivingSparkFlex, 8);

    drivingSparkFlex.burnFlash();
    turningSparkMax.burnFlash();
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(drivingEncoder.getVelocity(),
        new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        drivingEncoder.getPosition(),
        new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState, double accelerationMpsSq) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(turningEncoder.getPosition()));

    var arbFF = 0.0;
    arbFF = drivingMotorFeedforward.calculate(optimizedDesiredState.speedMetersPerSecond, accelerationMpsSq);
    drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkBase.ControlType.kVelocity, 0, arbFF, ArbFFUnits.kVoltage);

    turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkBase.ControlType.kPosition);

    this.desiredState = desiredState;
  }

  //Required for functionality of other classes
  public void setDesiredState(SwerveModuleState desiredState){
    setDesiredState(desiredState, 0);
  }
  
  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    drivingEncoder.setPosition(0);
  }
  
  public void setCurrentLimit(double amps){
    drivingSparkFlex.setSmartCurrentLimit((int)amps);
  }

  public void setVoltageDrive(Measure<Voltage> voltage){
    drivingSparkFlex.setVoltage(voltage.in(Units.Volts));

    turningPIDController.setReference(0, CANSparkMax.ControlType.kPosition);
  }

  public double getPowerOutput(){
    return drivingSparkFlex.getAppliedOutput() * drivingSparkFlex.getBusVoltage();
  }

  public double getDriveCurrent(){
    return drivingSparkFlex.getOutputCurrent();
  }

  public double getDrivingEncoderPosition(){
    return drivingEncoder.getPosition();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("MaxSwerveModule");
    builder.addDoubleProperty("Azimuth Angle", turningEncoder::getPosition, null);
    builder.addDoubleProperty("Encoder Dist", drivingEncoder::getPosition, drivingEncoder::setPosition);
  }

  public SwerveModuleState getDesiredSate(){
    return desiredState;
  }
}
