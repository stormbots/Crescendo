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

public class ShooterVision extends SubsystemBase {
  /** Creates a new Vision. */
  private AHRS gyro;

  public enum LimelightPipeline {
    kNoZoom, kZoom
  }

  NetworkTable camera = NetworkTableInstance.getDefault().getTable("limelight");
  
  NetworkTableEntry bpTable = camera.getEntry("botpose"); //gets translation (x, y, z) and rotation (x, y, z) for bot pose; may or may not change; currently gets bp relative to the target
  Pose2d botPose = new Pose2d(0, 0, new Rotation2d());
  private SwerveDrivePoseEstimator poseEstimator;
  //is there a way we can cross check the two bot poses????

  public double bpDefault [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  public Rotation2d rot = new Rotation2d(0,0);
  public double llTargetValid = 0.0;
  public double targetHeight = 48.0;
  public double camHeight = 6.0;
  public double camAngle = 80.0; //degrees
  private double horizontalOffset = 0.0;
  private double verticalOffset = 0.0;

  Field2d field = new Field2d();

  public ShooterVision(AHRS gyro, SwerveDrivePoseEstimator poseEstimator) { //need to add pose estimator
    this.gyro = gyro;
    this.poseEstimator = poseEstimator;
    setPipeline(LimelightPipeline.kNoZoom);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  
    
    double tv = camera.getEntry("tv").getDouble(0.0);
    var valid = tv >=1;
    if (! valid) {return;}
    double[] bp = bpTable.getDoubleArray(bpDefault);

    // var target = getDistanceAprilTag();
    // if(target.isEmpty()){return;}
    // SmartDashboard.putNumber("distance", target.get());
    // SmartDashboard.putNumber("verticalOffset", verticalOffset);
    var distance = getDistanceAprilTag(); //meters
    SmartDashboard.putNumber("limelight/translation2d", -distance.get()); //distance is a negative for some reason

    ifZoom();
    rot = new Rotation2d( Math.toRadians(bp[5]) );
    botPose = new Pose2d(bp[0]+15.980/2.0, bp[1]+8.210/2.0, rot);
    poseEstimator.addVisionMeasurement(botPose, Timer.getFPGATimestamp());
    field.getRobotObject().setPose(poseEstimator.getEstimatedPosition());
    field.getObject("visionpose").setPose(botPose);
    SmartDashboard.putData("shootervision", field);
  }

  public Optional<Double> getDistanceAprilTag() {
    double[] bp = camera.getEntry("botpose_targetspace").getDoubleArray(bpDefault);
    if (Array.getLength(bp)<6) {return Optional.empty();} //should work?

    Double distance = bp[2]; //meters
    return Optional.of(distance); //negative (idk why)
  }

  public Optional<Double> getDistanceOdometry(Pose3d target) {
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
      camera.getEntry("pipeline").setNumber(0);
      break;
      case kZoom:
      camera.getEntry("pipeline").setNumber(1);
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
  
  public double getTargetHeading() {
    return gyro.getAngle() + horizontalOffset;
  }

  // public double getHorizontalOffset() {
  //   return horizontalOffset;
  // }

  // public double getVerticalOffset() {
  //   return verticalOffset;
  // }

  public Optional<double[]> getCrosshairOffset() {
    double[] bp = bpTable.getDoubleArray(bpDefault);
    if (Array.getLength(bp)<6) {return Optional.empty();} //should work?
    horizontalOffset = camera.getEntry("tx").getDouble(0.0);
    verticalOffset = camera.getEntry("ty").getDouble(0.0);
    double[] offset = {horizontalOffset, verticalOffset};
    return Optional.of(offset);
  }
  public void ifZoom() {
    double tx = camera.getEntry("tx").getDouble(0.0);
    double ty = camera.getEntry("ty").getDouble(0.0);
    if ((tx<=11.0&&tx>=-11.0) && (ty<=5.0&&tx>=-5)) {
      setPipeline(ShooterVision.LimelightPipeline.kZoom);
    }
    else {
      setPipeline(ShooterVision.LimelightPipeline.kNoZoom);
    }
  }
}
