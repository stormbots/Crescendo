// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/**
 * NOTE: To use the Spark Flex / NEO Vortex, replace all instances of "CANSparkMax" with
 * "CANSparkFlex".
 */
public class DriveIOSparkMax implements DriveIO {
  private static final double gearRatio = 1.0;
  private static final double kP = 0.05; // TODO: MUST BE TUNED, consider using REV Hardware Client
  private static final double kD = 0.0; // TODO: MUST BE TUNED, consider using REV Hardware Client

  private final CANSparkMax leftMotor = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax rightMotor = new CANSparkMax(2, MotorType.kBrushless);
  
  private final RelativeEncoder leftEncoder = leftMotor.getEncoder();
  private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
  private final SparkPIDController leftPID = leftMotor.getPIDController();
  private final SparkPIDController rightPID = rightMotor.getPIDController();

  // private final Pigeon2 pigeon = new Pigeon2(20);
  // private final StatusSignal<Double> yaw = pigeon.getYaw();

  public DriveIOSparkMax() {
    leftMotor.restoreFactoryDefaults();
    rightMotor.restoreFactoryDefaults();
    

    leftMotor.setCANTimeout(250);
    rightMotor.setCANTimeout(250);
    

    leftMotor.setInverted(false);
    rightMotor.setInverted(true);
    

    leftMotor.enableVoltageCompensation(12.0);
    rightMotor.enableVoltageCompensation(12.0);
    leftMotor.setSmartCurrentLimit(60);
    rightMotor.setSmartCurrentLimit(60);

    leftPID.setP(kP);
    leftPID.setD(kD);
    rightPID.setP(kP);
    rightPID.setD(kD);

    leftMotor.burnFlash();
    rightMotor.burnFlash();
    

    // pigeon.getConfigurator().apply(new Pigeon2Configuration());
    // pigeon.getConfigurator().setYaw(0.0);
    // yaw.setUpdateFrequency(100.0);
    // pigeon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    inputs.leftPositionRad = Units.rotationsToRadians(leftEncoder.getPosition() / gearRatio);
    inputs.leftVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(leftEncoder.getVelocity() / gearRatio);
    inputs.leftAppliedVolts = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
    inputs.leftCurrentAmps =
        new double[] {leftMotor.getOutputCurrent()};

    inputs.rightPositionRad = Units.rotationsToRadians(rightEncoder.getPosition() / gearRatio);
    inputs.rightVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(rightEncoder.getVelocity() / gearRatio);
    inputs.rightAppliedVolts = rightMotor.getAppliedOutput() * rightMotor.getBusVoltage();
    inputs.rightCurrentAmps =
        new double[] {rightMotor.getOutputCurrent()};

    // inputs.gyroYaw = Rotation2d.fromDegrees(yaw.refresh().getValueAsDouble());
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    leftMotor.setVoltage(leftVolts);
    rightMotor.setVoltage(rightVolts);
  }

  @Override
  public void setVelocity(
      double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {
    leftPID.setReference(
        Units.radiansPerSecondToRotationsPerMinute(leftRadPerSec * gearRatio),
        ControlType.kVelocity,
        0,
        leftFFVolts);
    rightPID.setReference(
        Units.radiansPerSecondToRotationsPerMinute(rightRadPerSec * gearRatio),
        ControlType.kVelocity,
        0,
        rightFFVolts);
  }
}
