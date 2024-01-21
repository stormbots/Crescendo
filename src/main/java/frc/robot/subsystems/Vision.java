// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Array;
import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private AHRS gyro;

  public enum LimelightPipeline {
    kNoZoom, kZoom
  }

  NetworkTable frontCamera = NetworkTableInstance.getDefault().getTable("front limelight");
  NetworkTable backCamera = NetworkTableInstance.getDefault().getTable("back limelight");
  
  NetworkTableEntry bpTableFront = frontCamera.getEntry("botpose front"); //gets translation (x, y, z) and rotation (x, y, z) for bot pose; may or may not change
  NetworkTableEntry bpTableBack = backCamera.getEntry("botpose back");
  Pose2d botPose = new Pose2d(0, 0, new Rotation2d());
  private SwerveDrivePoseEstimator poseEstimator;
  //is there a way we can cross check the two bot poses????

  public double bpDefault [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  public Rotation2d rotFront = new Rotation2d(0,0);

  Field2d field = new Field2d();

  public Vision(AHRS gyro, SwerveDrivePoseEstimator poseEstimator) { //need to add pose estimator
    this.gyro = gyro;
    this.poseEstimator = poseEstimator;
    setPipeline(LimelightPipeline.kNoZoom);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double xFront = frontCamera.getEntry("x").getDouble(0);
    double yFront = frontCamera.getEntry("y").getDouble(0);

    double xBack = backCamera.getEntry("x").getDouble(0);
    double yBack = backCamera.getEntry("y").getDouble(0);

    double[] bpFront = bpTableFront.getDoubleArray(bpDefault);
    if (Array.getLength(bpFront)<6) {return;}

    double[] bpBack = bpTableBack.getDoubleArray(bpDefault);
    if (Array.getLength(bpBack)<6) {return;}

    rotFront = new Rotation2d( Math.toRadians(bpFront[5]) );

    poseEstimator.addVisionMeasurement(botPose, Timer.getFPGATimestamp());
    //TODO: chassis stuff
  }

  public Optional<Double> getDistanceAprilTag() { //or use Translation2d
    //TODO: if has value return non-empty but no values rn :( (april tag)
    return Optional.empty();
  }

  public Optional<Double> getDistanceOdometry() {
    //TODO: if has value return non-empty but no values rn :( (odometry)
    return Optional.empty();
  }
  public Optional<Double> getAngleToNote() {
    //TODO: if has value return non-empty but no values rn :( (angle to note)
    return Optional.empty();
  }

  public Pose2d getBotPose() { 
    return botPose;
  }

  public void setPipeline(LimelightPipeline pipeline) {
    switch(pipeline) {
      case kNoZoom:
      frontCamera.getEntry("pipeline").setNumber(0);
      break;
      case kZoom:
      frontCamera.getEntry("pipeline").setNumber(1);
    }
  }
}
