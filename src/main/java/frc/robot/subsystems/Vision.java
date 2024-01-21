// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Array;
import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private AHRS gyro;

  public enum LimelightPipeline {
    kNoZoom, kZoom
  }

  NetworkTable frontCamera = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTable backCamera = NetworkTableInstance.getDefault().getTable("back limelight");
  
  NetworkTableEntry bpTableFront = frontCamera.getEntry("botpose_targetspace"); //gets translation (x, y, z) and rotation (x, y, z) for bot pose; may or may not change
  NetworkTableEntry bpTableBack = backCamera.getEntry("botpose back");
Pose2d botPoseFront = new Pose2d(0, 0, new Rotation2d());
  private SwerveDrivePoseEstimator poseEstimator;
  //is there a way we can cross check the two bot poses????

  public double bpDefault [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  public Rotation2d rotFront = new Rotation2d(0,0);
  public double verticalOffset = 0.0;
  public double llTargetValid = 0.0;
  public double targetHeight = 48.0;
  public double camHeight = 6.0;
  public double camAngle = 80.0; //degrees

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
    double[] bpFront = bpTableFront.getDoubleArray(bpDefault);
    if (Array.getLength(bpFront)<6) {return;}
    rotFront = new Rotation2d( Math.toRadians(bpFront[5]) );

    double xBack = backCamera.getEntry("x").getDouble(0);
    double yBack = backCamera.getEntry("y").getDouble(0);
    verticalOffset = frontCamera.getEntry("ty").getDouble(0.0);
    double[] bpBack = bpTableBack.getDoubleArray(bpDefault);
    if (Array.getLength(bpBack)<6) {return;}

    // var target = getDistanceAprilTag();
    // if(target.isEmpty()){return;}
    // SmartDashboard.putNumber("distance", target.get());
    // SmartDashboard.putNumber("verticalOffset", verticalOffset);
    var distance = getDistanceAprilTag(); //meters
    SmartDashboard.putNumber("limelight/translation2d", -distance.get()); //distance is a negative for some reason

    rotFront = new Rotation2d( Math.toRadians(bpFront[5]) );

    poseEstimator.addVisionMeasurement(botPoseFront, Timer.getFPGATimestamp());
    //TODO: chassis stuff
  }

  public Optional<Double> getDistanceAprilTag() { //use Translation2d, doesn't work
    double[] bpFront = bpTableFront.getDoubleArray(bpDefault); //just for front
    if (Array.getLength(bpFront)<6) {return Optional.empty();} //should work?

    Double distance = bpFront[2]; //meters
    return Optional.of(distance); //negative (idk why)
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
    return botPoseFront;
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

  public double getAngleToTargetPose(Pose3d pose) {
    Pose2d botPose = poseEstimator.getEstimatedPosition();
    var targetPose = pose.toPose2d();

    double dx = targetPose.getX() - botPose.getX();
    double dy = targetPose.getY() - botPose.getY();

    double angle = Math.toDegrees(Math.atan2(dy,dx));

    double botPoseAngle = (botPose.getRotation().getDegrees() % 360);
    angle = botPoseAngle - angle;
    return angle;
  }
}
