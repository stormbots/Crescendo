// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Chassis;

import java.util.OptionalDouble;
import java.util.Queue;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

public class NavX implements GyroIO {
  private final AHRS navx = new AHRS();
  private float yaw = navx.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final double yawVelocity = navx.getRate();

  public NavX() {
    navx.reset();
      yawTimestampQueue = OdometryThread.getInstance().makeTimestampQueue();
      yawPositionQueue =
          OdometryThread.getInstance()
              .registerSignal(
                  () -> {
                    boolean valid = (navx.isConnected());
                    if (valid) {
                      return OptionalDouble.of(navx.getYaw());
                    } else {
                      return OptionalDouble.empty();
                    }
                  });
    }
  

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    yaw = navx.getYaw();
    inputs.connected = navx.isConnected();
    inputs.yawPosition = Rotation2d.fromDegrees(yaw);
    inputs.yawVelocityDegPerSec = Units.degreesToRadians(yawVelocity);

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
