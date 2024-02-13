// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Clamp;

public class ShooterVision extends SubsystemBase {
  /** Creates a new Vision. */
  private AHRS gyro;

  public enum LimelightPipeline {
    kNoVision, kNoZoom, kZoom
  }
  public class LimelightReadings {
    public double targetID;
    public double distance; //meters
    public double angleHorizontal; //degrees
    public double angleVertical; //degrees
    public Double time;
  }

  NetworkTable camera = NetworkTableInstance.getDefault().getTable("limelight");
  
  //NetworkTableEntry bpTable = camera.getEntry("botpose"); //gets translation (x, y, z) and rotation (x, y, z) for bot pose
  //public double bpDefault [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  //public Pose2d botPose;
  //public Rotation2d rot;
  Field2d field = new Field2d();
  //public LimelightReadings limelightReadings;
  public SwerveDrivePoseEstimator poseEstimator;

  //public double targetHeight = Units.inchesToMeters(48.0);
  //public double camHeight = Units.inchesToMeters(6);
  //public double camAngle = 80.0; //degrees

  public ShooterVision(AHRS gyro, SwerveDrivePoseEstimator poseEstimator) { //need to add pose estimator
    this.gyro = gyro;
    this.poseEstimator = poseEstimator;
    
    setPipeline(LimelightPipeline.kNoZoom);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("vision/rotations in radians", poseEstimator.getEstimatedPosition().getRotation().getRadians());
    // SmartDashboard.putNumber("vision/x", poseEstimator.getEstimatedPosition().getX());
    // SmartDashboard.putNumber("vision/y", poseEstimator.getEstimatedPosition().getY());
    if (hasValidTarget()==false) {return;}

    zoomIfPossible();
    updateOdometry();

    SmartDashboard.putData("visionfield", field);
  }

  public boolean hasValidTarget() {
    double tv = camera.getEntry("tv").getDouble(0.0);
    return tv >= 1;
  }

  public Optional<LimelightReadings> getVisibleTarget() {
    if (hasValidTarget()==false) {return Optional.empty();}

    double[] bp = camera.getEntry("botpose_targetspace").getDoubleArray(new double[]{0,0,0,0,0,0});

    var target = new LimelightReadings();
    target.targetID = camera.getEntry("tid").getDouble(0.0);
    target.distance = bp[2]; //meters TODO: negative for some reason :(
    target.angleHorizontal = camera.getEntry("tx").getDouble(0.0);
    target.angleVertical = camera.getEntry("ty").getDouble(0.0);
    target.time = Timer.getFPGATimestamp();

    return Optional.of(target);
  }

  private void updateOdometry() {
    if (getVisibleTarget().isEmpty()) {return;}

    double[] bp = camera.getEntry("botpose").getDoubleArray(new double[]{0,0,0,0,0,0});

    Rotation2d rot = new Rotation2d(Math.toRadians(bp[5]));
    Pose2d botPose = new Pose2d(bp[0]+15.980/2.0, bp[1]+8.210/2.0, rot);
    poseEstimator.addVisionMeasurement(botPose, Timer.getFPGATimestamp());

    var stdevs = new Matrix<>(Nat.N3(), Nat.N1(), new double[]{1,1,1}); //confidence checker
    poseEstimator.setVisionMeasurementStdDevs(stdevs);

    field.getRobotObject().setPose(poseEstimator.getEstimatedPosition());
    field.getObject("visionpose").setPose(botPose);
  }

  public void zoomIfPossible() {
    var target = getVisibleTarget();
    if (hasValidTarget()==false) {return;}

    double tx = target.get().angleHorizontal;
    double ty = target.get().angleVertical;
    if (Clamp.bounded(tx, -11, 11) && Clamp.bounded(ty, -10, 10)) {
      setPipeline(LimelightPipeline.kZoom);
    }
    else {
      setPipeline(LimelightPipeline.kNoZoom);
    }
  }

  public void setPipeline(LimelightPipeline pipeline) {
    switch(pipeline) {
      case kNoVision:
      camera.getEntry("pipeline").setNumber(0);
      case kNoZoom:
      camera.getEntry("pipeline").setNumber(1);
      break;
      case kZoom:
      camera.getEntry("pipeline").setNumber(2);
    }
  }
}