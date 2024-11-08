// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Chassis;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkFlexFixes;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.SparkPIDController;
import frc.robot.subsystems.Chassis.ChassisConstants.DriveConstants;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Chassis.ChassisConstants.ModuleConstants;
import frc.robot.subsystems.Chassis.ModuleIO.ModuleIOInputs;

public class ModuleIOMAXSwerve implements ModuleIO{
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
  public ModuleIOMAXSwerve(int index){
    switch (index) {
      case 0:
        drivingSparkFlex = new CANSparkFlex(DriveConstants.kFrontRightDrivingCanId, MotorType.kBrushless);
        turningSparkMax = new CANSparkMax(DriveConstants.kFrontRightTurningCanId, MotorType.kBrushless);
        break;
      case 1:
        drivingSparkFlex = new CANSparkFlex(DriveConstants.kRearRightDrivingCanId, MotorType.kBrushless);
        turningSparkMax = new CANSparkMax(DriveConstants.kRearRightTurningCanId, MotorType.kBrushless);
        break;
      case 2:
        drivingSparkFlex = new CANSparkFlex(DriveConstants.kFrontLeftDrivingCanId, MotorType.kBrushless);
        turningSparkMax = new CANSparkMax(DriveConstants.kFrontLeftTurningCanId, MotorType.kBrushless);
        break;
      case 3:
        drivingSparkFlex = new CANSparkFlex(DriveConstants.kRearLeftDrivingCanId, MotorType.kBrushless);
        turningSparkMax = new CANSparkMax(DriveConstants.kRearLeftTurningCanId, MotorType.kBrushless);
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

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

    drivingSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 5);
    drivingSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    drivingSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
    drivingSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 1000);
    drivingSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);
    // drivingSparkFlex.setClosedLoopRampRate(0.02);

    turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 5);
    turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    
    // SparkFlexFixes.setFlexEncoderAverageDepth(drivingSparkFlex, 2);
    // SparkFlexFixes.setFlexEncoderSampleDelta(drivingSparkFlex, 8);

    drivingSparkFlex.burnFlash();
    turningSparkMax.burnFlash();

    
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad =
        (drivingEncoder.getPosition()*2*(Math.PI)) / ModuleConstants.kDrivingMotorReduction;
    inputs.driveVelocityRadPerSec =
        (drivingEncoder.getVelocity()*(Math.PI/30)) / ModuleConstants.kDrivingMotorReduction;
    inputs.driveAppliedVolts = drivingSparkFlex.getAppliedOutput() * drivingSparkFlex.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {drivingSparkFlex.getOutputCurrent()};

    inputs.modulePosition =
    new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset);
    inputs.turnPosition =
        (new Rotation2d(turningEncoder.getPosition()));
    inputs.turnVelocityRadPerSec =
        (turningEncoder.getVelocity()*2*(Math.PI));
    inputs.turnAppliedVolts = turningSparkMax.getAppliedOutput() * turningSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turningSparkMax.getOutputCurrent()};
  }

  @Override
  public void setDriveVoltage(double volts) {
    drivingSparkFlex.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turningSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    drivingSparkFlex.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turningSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
  
}
