// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Chassis;

import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Chassis.ChassisConstants.AdvantageKitConstants;

public class OdometryThread extends SubsystemBase {
  private List<Supplier<OptionalDouble>> signals = new ArrayList<>();
  private List<Queue<Double>> queues = new ArrayList<>();
  private List<Queue<Double>> timestampQueues = new ArrayList<>();

  private final Notifier notifier;
  private static OdometryThread instance = null;

  public static OdometryThread getInstance() {
    if (instance == null) {
      instance = new OdometryThread();
    }
    return instance;
  }

  private OdometryThread() {
    notifier = new Notifier(this::periodic);
    notifier.setName("OdometryThread");
  }

  public void start() {
    if (timestampQueues.size() > 0) {
      notifier.startPeriodic(1.0 / AdvantageKitConstants.kodometryFrequency);
    }
  }

  public Queue<Double> registerSignal(Supplier<OptionalDouble> signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Chassis.odometryLock.lock();
    try {
      signals.add(signal);
      queues.add(queue);
    } finally {
      Chassis.odometryLock.unlock();
    }
    return queue;
  }

  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Chassis.odometryLock.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      Chassis.odometryLock.unlock();
    }
    return queue;
  }

  public void periodic() {
    Chassis.odometryLock.lock();
    double timestamp = Logger.getRealTimestamp() / 1e6;
    try {
      double[] values = new double[signals.size()];
      boolean isValid = true;
      for (int i = 0; i < signals.size(); i++) {
        OptionalDouble value = signals.get(i).get();
        if (value.isPresent()) {
          values[i] = value.getAsDouble();
        } else {
          isValid = false;
          break;
        }
      }
      if (isValid) {
        for (int i = 0; i < queues.size(); i++) {
          queues.get(i).offer(values[i]);
        }
        for (int i = 0; i < timestampQueues.size(); i++) {
          timestampQueues.get(i).offer(timestamp);
        }
      }
    } finally {
      Chassis.odometryLock.unlock();
    }
  }
}
