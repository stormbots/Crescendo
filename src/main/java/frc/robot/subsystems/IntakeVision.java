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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeVision extends SubsystemBase {
  /** Creates a new Vision. */
  private AHRS gyro;

  public enum LimelightPipeline {
    kNoZoom, kZoom
  }
  public class LimelightReadings {
    public double xDegreesOffset;
    public double yDegreesOffset;
    private double targetValid;

    public LimelightReadings(double xDegreesOffset, double yDegreesOffset, double targetValid) {
      this.xDegreesOffset = xDegreesOffset;
      this.yDegreesOffset = yDegreesOffset;
      this.targetValid = targetValid;
    }

    public boolean isTargetValid() {
      if (targetValid>=1) {
        return true;
      }
      return false;
    }
  }

  NetworkTable camera = NetworkTableInstance.getDefault().getTable("limelight");
  
  NetworkTableEntry bpTable = camera.getEntry("botpose"); //gets translation (x, y, z) and rotation (x, y, z) for bot pose
  public double bpDefault [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  public Pose2d botPose;
  public Rotation2d rot;
  Field2d field = new Field2d();
  public LimelightReadings limelightReadings;
  private SwerveDrivePoseEstimator poseEstimator;

  public double targetHeight = Units.inchesToMeters(48.0);
  public double camHeight = Units.inchesToMeters(6);
  public double camAngle = 80.0; //degrees

  public IntakeVision(AHRS gyro, SwerveDrivePoseEstimator poseEstimator) { //need to add pose estimator
    this.gyro = gyro;
    this.poseEstimator = poseEstimator;
    
    setPipeline(LimelightPipeline.kNoZoom);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    limelightReadings = new LimelightReadings(
      camera.getEntry("tx").getDouble(0.0),
      camera.getEntry("ty").getDouble(0.0), 
      camera.getEntry("tv").getDouble(0.0)
    );
    
    if (!limelightReadings.isTargetValid()) {return;}

    double[] bp = bpTable.getDoubleArray(bpDefault);
    ifZoom();

    rot = new Rotation2d(Math.toRadians(bp[5]));
    botPose = new Pose2d(bp[0]+15.980/2.0, bp[1]+8.210/2.0, rot);
    poseEstimator.addVisionMeasurement(botPose, Timer.getFPGATimestamp());

    var stdevs = new Matrix<>(Nat.N3(), Nat.N1(), new double[]{1,1,1});
    poseEstimator.setVisionMeasurementStdDevs(stdevs);

    field.getRobotObject().setPose(poseEstimator.getEstimatedPosition());
    field.getObject("visionpose").setPose(botPose);
    SmartDashboard.putData("visionfield", field);
  }

  public Optional<Double> getDistance() {
    double[] bp = camera.getEntry("botpose_targetspace").getDoubleArray(bpDefault);
    if (limelightReadings.isTargetValid()) {return Optional.empty();}

    Double distance = bp[2]; //meters
    return Optional.of(distance); //negative on test bench
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

  public double getTargetHeading() {
    return gyro.getAngle() + limelightReadings.xDegreesOffset;
  }

  public Optional<double[]> getCrosshairOffset() {
    if (!limelightReadings.isTargetValid()) {return Optional.empty();} //should work?
    double[] offset = {limelightReadings.xDegreesOffset, limelightReadings.yDegreesOffset};
    return Optional.of(offset);
  }

  public void ifZoom() {
    double xDegreesOffset = limelightReadings.xDegreesOffset;
    double yDegreesOffset = limelightReadings.yDegreesOffset;
    if ((xDegreesOffset<=11.0&&xDegreesOffset>=-11.0) && (yDegreesOffset<=10.0&&yDegreesOffset>=-10.0)) {
      setPipeline(IntakeVision.LimelightPipeline.kZoom);
    }
    else {
      setPipeline(IntakeVision.LimelightPipeline.kNoZoom);
    }
  }

  public void setPipeline(LimelightPipeline pipeline) {
    switch(pipeline) {
      case kNoZoom:
      camera.getEntry("pipeline").setNumber(0);
      break;
      case kZoom:
      camera.getEntry("pipeline").setNumber(1);
    }
  }
}