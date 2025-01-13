// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.ChassisConstants.ModuleConstants;

public class MAXSwerveModule implements Sendable{
  public final SparkFlex drivingSparkFlex;
  private final SparkMax turningSparkMax;

  public final RelativeEncoder drivingEncoder;
  public final AbsoluteEncoder turningEncoder;

  private final SparkClosedLoopController drivingPIDController;
  private final SparkClosedLoopController turningPIDController;

  private final SparkBaseConfig drivingConfig = new SparkFlexConfig();
  private final SparkBaseConfig turningConfig = new SparkMaxConfig();

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
    drivingSparkFlex = new SparkFlex(drivingCANId, MotorType.kBrushless);
    turningSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);
    this.drivingMotorFeedforward = drivingMotorFeedforward;

    drivingSparkFlex.clearFaults();
    turningSparkMax.clearFaults();

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    // drivingSparkFlex.restoreFactoryDefaults();
    // turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    drivingEncoder = drivingSparkFlex.getEncoder();
    turningEncoder = turningSparkMax.getAbsoluteEncoder();
    drivingPIDController = drivingSparkFlex.getClosedLoopController();
    turningPIDController = turningSparkMax.getClosedLoopController();
    drivingConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    turningConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.
    drivingConfig.encoder.positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor);
    drivingConfig.encoder.velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);
    
    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    turningConfig.absoluteEncoder.positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    turningConfig.absoluteEncoder.velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    turningConfig.absoluteEncoder.inverted(ModuleConstants.kTurningEncoderInverted);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    turningConfig.closedLoop.positionWrappingEnabled(true)
    .positionWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput)
    .positionWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    drivingConfig.closedLoop
    .p(ModuleConstants.kDrivingP)
    .i(ModuleConstants.kDrivingI)
    .i(ModuleConstants.kDrivingD)
    // drivingPIDController.setFF(ModuleConstants.kDrivingFF);  
    .outputRange(ModuleConstants.kDrivingMinOutput,ModuleConstants.kDrivingMaxOutput);

    // Set the PID gains for the turning motor. Note these are example gains, and you
    // may need to tune them for your own robot!
    turningConfig.closedLoop
    .p(ModuleConstants.kTurningP)
    .i(ModuleConstants.kTurningI)
    .d(ModuleConstants.kTurningD)
    .velocityFF(ModuleConstants.kTurningFF)
    .outputRange(ModuleConstants.kTurningMinOutput,
        ModuleConstants.kTurningMaxOutput);

    drivingConfig.idleMode(IdleMode.kBrake);
    turningConfig.idleMode(IdleMode.kBrake);
    drivingConfig.smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
    turningConfig.smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    drivingConfig.encoder.uvwAverageDepth(2);
    drivingConfig.encoder.quadratureAverageDepth(2);
    drivingConfig.encoder.quadratureMeasurementPeriod(8);
    // SparkFlexFixes.setFlexEncoderAverageDepth(drivingSparkFlex, 2);
    // SparkFlexFixes.setFlexEncoderSampleDelta(drivingSparkFlex, 8);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.

    this.chassisAngularOffset = chassisAngularOffset;
    desiredState.angle = new Rotation2d(turningEncoder.getPosition());
    drivingEncoder.setPosition(0);

    drivingConfig.apply(Constants.kSwerveDrive);
    turningConfig.apply(Constants.kSwerveSteering);
    
    drivingSparkFlex.configure(drivingConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    turningSparkMax.configure(turningConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    // drivingSparkFlex.burnFlash();
    // turningSparkMax.burnFlash();
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
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(turningEncoder.getPosition()));

    var arbFF = 0.0;
    if(DriverStation.isAutonomous()){
      arbFF= drivingMotorFeedforward.calculateWithVelocities(drivingEncoder.getVelocity(), optimizedDesiredState.speedMetersPerSecond);
      // I don't remember what the 0.08 intends to do
      // arbFF = drivingMotorFeedforward.calculate(drivingEncoder.getVelocity(), optimizedDesiredState.speedMetersPerSecond, 0.08);
    }
    else{
      arbFF = drivingMotorFeedforward.calculate(optimizedDesiredState.speedMetersPerSecond);
    }
    drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, ControlType.kVelocity, ClosedLoopSlot.kSlot0, arbFF, ArbFFUnits.kVoltage);

    turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), ControlType.kPosition);

    this.desiredState = desiredState;
  }
  
  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    drivingEncoder.setPosition(0);
  }
  
  public void setCurrentLimit(double amps){
    var config=new SparkFlexConfig();
    config.smartCurrentLimit((int)amps);
    drivingSparkFlex.configureAsync(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setVoltageDrive(Voltage voltage){
    drivingSparkFlex.setVoltage(voltage.in(Units.Volts));

    turningPIDController.setReference(0, ControlType.kPosition);
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
