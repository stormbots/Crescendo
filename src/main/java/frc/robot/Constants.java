// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SignalsConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  /**
   * Allows velocity, position, and follower feedback
   */
  public final static SignalsConfig kTypical = new SignalsConfig()
  .absoluteEncoderPositionAlwaysOn(false)
  .absoluteEncoderVelocityAlwaysOn(false)
  .analogVelocityAlwaysOn(false)
  .analogPositionAlwaysOn(false)
  .analogVoltageAlwaysOn(false)
  .externalOrAltEncoderPositionAlwaysOn(false)
  .externalOrAltEncoderVelocityAlwaysOn(false)
  .iAccumulationAlwaysOn(false);
  ;
  // Attempt 1: Just disable the unnecessary signals 


  //  https://docs.revrobotics.com/brushless/spark-max/control-interfaces
  // motorB.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 200);
  //critical, don't touch
  // motorB.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);
  // velocity, temp, voltage, current
  // motorB.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 200);
  // position

  // motorB.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
  // analog voltage, velocity, position
  // motorB.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
  // alt encoder position, velocity

  // motorB.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 1000);
  // duty cycle encoder position/absolute angle
  // motorB.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);
  // duty cycle encoder velocity, encoder frequency 


  /**
   * Allow other encoders, but disable the attached absolute encoder
   */ 
  public final static SignalsConfig kAbsEncoder = new SignalsConfig()
  .apply(kTypical) //get rid of the usual stuff
  //Re-enable the required absolute encoders
  .absoluteEncoderPositionAlwaysOn(true)
  .absoluteEncoderPositionPeriodMs(10)
  .absoluteEncoderVelocityPeriodMs(10)
  ;
  //here's the config for absolute encoder ones from shooter
  // shooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
  // shooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
  // shooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 10);
  // shooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);

  public final static SignalsConfig kSwerveSteering = new SignalsConfig()
  .apply(kTypical) //get rid of the usual stuff
  .absoluteEncoderPositionAlwaysOn(true)
  .absoluteEncoderPositionPeriodMs(5)
  .primaryEncoderVelocityAlwaysOn(true)
  .primaryEncoderVelocityPeriodMs(5)
  ;
  public final static SignalsConfig kSwerveDrive = new SignalsConfig()
  .apply(kAbsEncoder) //get rid of the usual stuff
  .primaryEncoderVelocityAlwaysOn(true)
  .primaryEncoderVelocityPeriodMs(5)
  .primaryEncoderPositionPeriodMs(5)
  ;

  // drivingSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 5);
  // drivingSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
  // drivingSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
  // drivingSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 1000);
  // drivingSparkFlex.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);
  // // drivingSparkFlex.setClosedLoopRampRate(0.02);

  // turningSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 5);

}
