// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;

public class DriveIOSim implements DriveIO {
  private static final double KP = 0.2;
  private static final double KD = 0.0;
  private DifferentialDrivetrainSim sim =
      DifferentialDrivetrainSim.createKitbotSim(
          KitbotMotor.kDualCIMPerSide, KitbotGearing.k7p31, KitbotWheelSize.kSixInch, null);

  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;
  private boolean closedLoop = false;
  private PIDController leftPID = new PIDController(KP, 0.0, KD);
  private PIDController rightPID = new PIDController(KP, 0.0, KD);
  private double leftFFVolts = 0.0;
  private double rightFFVolts = 0.0;

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    if (closedLoop) {
      leftAppliedVolts =
          MathUtil.clamp(
              leftPID.calculate(sim.getLeftVelocityMetersPerSecond() / Drive.wheelRadius)
                  + leftFFVolts,
              -12.0,
              12.0);
      rightAppliedVolts =
          MathUtil.clamp(
              leftPID.calculate(sim.getRightVelocityMetersPerSecond() / Drive.wheelRadius)
                  + rightFFVolts,
              -12.0,
              12.0);
      sim.setInputs(leftAppliedVolts, rightAppliedVolts);
    }

    sim.update(0.02);
    inputs.leftPositionRad = sim.getLeftPositionMeters() / Drive.wheelRadius;
    inputs.leftVelocityRadPerSec = sim.getLeftVelocityMetersPerSecond() / Drive.wheelRadius;
    inputs.leftAppliedVolts = leftAppliedVolts;
    inputs.leftCurrentAmps = new double[] {sim.getLeftCurrentDrawAmps()};

    inputs.rightPositionRad = sim.getRightPositionMeters() / Drive.wheelRadius;
    inputs.rightVelocityRadPerSec = sim.getRightVelocityMetersPerSecond() / Drive.wheelRadius;
    inputs.rightAppliedVolts = rightAppliedVolts;
    inputs.rightCurrentAmps = new double[] {sim.getRightCurrentDrawAmps()};

    inputs.gyroYaw = sim.getHeading();
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    closedLoop = false;
    leftAppliedVolts = MathUtil.clamp(leftVolts, -12.0, 12.0);
    rightAppliedVolts = MathUtil.clamp(rightVolts, -12.0, 12.0);
    sim.setInputs(leftAppliedVolts, rightAppliedVolts);
  }

  @Override
  public void setVelocity(
      double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {
    closedLoop = true;
    leftPID.setSetpoint(leftRadPerSec);
    rightPID.setSetpoint(rightRadPerSec);
    this.leftFFVolts = leftFFVolts;
    this.rightFFVolts = rightFFVolts;
  }
}
