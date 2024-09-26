// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ChassisConstants {
    
    public static final class DriveConstants {
       // Chassis configuration
      //NOTE: These are over-written in robot.java
      public static double kTrackWidth = Units.inchesToMeters(23.5);
      // Distance between centers of right and left wheels on robot
      public static  double kWheelBase = Units.inchesToMeters(23.5);
      // Distance between front and back wheels on robot
      public static double distanceToModuleFromCenter = Math.hypot(kTrackWidth/2, kWheelBase/2);

    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 5.4;
    // public static final double kMaxAngularSpeed = 4 * Math.PI; // radians per second
    public static final double kMaxAngularSpeed = kMaxSpeedMetersPerSecond / kTrackWidth * 2; // PREVIOUS VALUE IS UNREASONABLE THIS VALUE IS APPROX 2.8PI

    public static final double kDirectionSlewRate = 1.2*3; // radians per second
    public static final double kMagnitudeSlewRate = 1.8*5; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2*2*2*2.0; // percent per second (1 = 100%)

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = 0;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = 0;
    public static final double kBackRightChassisAngularOffset = 0;

    // SPARK FLEX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 4;
    public static final int kFrontRightDrivingCanId = 2;
    public static final int kRearRightDrivingCanId = 3;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftTurningCanId = 5;
    public static final int kRearLeftTurningCanId = 8;
    public static final int kFrontRightTurningCanId = 6;
    public static final int kRearRightTurningCanId = 7; 

  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762*0.975;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 20) / (kDrivingMotorPinionTeeth * 15);
    //Theoretical max speed, has nothing to do with rotation
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = kDrivingEncoderPositionFactor / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI); // radians per second uses absolute not relative

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps * 1.2;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.000;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3; //4.375
    //NOTE: Half of these are only used once, only use for them would be to improve code understanding, as to which these names rev provided are actually HORRIBLE

    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    // public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI;

  //   // Unused, from original template
    public static final double kPDrivingP = 2/1;
    public static final double kPThetaP = 0.01/360.0; //idk, very small value so it dont break anything too much

  //   // Constraint for the motion profiled robot angle controller
    // public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
    //     kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 6784;
  }
}
