// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.sql.Driver;
import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Clamp;

public class ShooterVision extends SubsystemBase {
  /** Creates a new Vision. */
  public enum LimelightPipeline {
    kNoVision, kOdometry, kZoom, kSpeaker
  }
  public class LimelightReadings {
    public Measure<Distance> distance; //inches
    public double angleHorizontal; //degrees
    public double angleVertical; //degrees
    public double time;
  }

  public NetworkTable camera = NetworkTableInstance.getDefault().getTable("limelight-shooter");
  Field2d field = new Field2d();
  FieldPosition fieldPos = new FieldPosition();
  public SwerveDrivePoseEstimator poseEstimator;

  public ShooterVision(SwerveDrivePoseEstimator poseEstimator){ //need to add pose estimator
    this.poseEstimator = poseEstimator;

    setPipeline(LimelightPipeline.kSpeaker);
  }

  @Override
  public void periodic() {
    if(DriverStation.isAutonomous()){return;}
    //zoomIfPossible(); pipeline makes frames drop a lot
    updateOdometry();
    if (hasValidTarget()) {SmartDashboard.putBoolean("shootervision/validtarget", true);}
    if (getVisibleTargetData().isPresent()) {SmartDashboard.putNumber("manualshoot/distance", getVisibleTargetData().get().distance.in(Units.Inches));}
    SmartDashboard.putData("shootervisionfield", field);
    SmartDashboard.putNumber("shootervision/tv", camera.getEntry("tv").getDouble(0.0));
  }

  public boolean hasValidTarget() {
    double tv = camera.getEntry("tv").getDouble(0.0);
    return (tv >= 1);
  }

  public Optional<LimelightReadings> getVisibleTargetData() {
    if (hasValidTarget()==false) {return Optional.empty();}

    double[] bp = camera.getEntry("targetpose_robotspace").getDoubleArray(new double[]{0,0,0,0,0,0});

    if (bp.length<6) {return Optional.empty();}
    SmartDashboard.putNumber("shootervision/testingdistance", bp[2]);
    //SmartDashboard.putData("shootervision", );
    var target = new LimelightReadings();
    target.distance = Units.Meters.of(bp[2]);
    target.angleHorizontal = camera.getEntry("tx").getDouble(0.0);
    target.angleVertical = camera.getEntry("ty").getDouble(0.0);
    target.time = Timer.getFPGATimestamp();

    return Optional.of(target);
  }

  private void updateOdometry() {
    if (hasValidTarget()==false) {return;}

    double[] bp = camera.getEntry("botpose").getDoubleArray(new double[]{0,0,0,0,0,0});

    if (bp.length<6) {return;}
    Rotation2d rot = new Rotation2d(Math.toRadians(bp[5]));
    Pose2d botPose = new Pose2d(bp[0]+15.980/2.0, bp[1]+8.210/2.0, rot);
    poseEstimator.addVisionMeasurement(botPose, Timer.getFPGATimestamp());

    var stdevs = new Matrix<>(Nat.N3(), Nat.N1(), new double[]{1,1,1}); //confidence checker
    poseEstimator.setVisionMeasurementStdDevs(stdevs);

    field.getRobotObject().setPose(poseEstimator.getEstimatedPosition());
    field.getObject("shootervisionpose").setPose(botPose);
  }

  public void zoomIfPossible() {
    var target = getVisibleTargetData();
    if (hasValidTarget()==false) {return;}

    double tx = target.get().angleHorizontal;
    double ty = target.get().angleVertical;
    if (Clamp.bounded(tx, -11, 11) && Clamp.bounded(ty, -10, 10)) {
      setPipeline(LimelightPipeline.kZoom);
    }
    else {
      setPipeline(LimelightPipeline.kOdometry);
    }
  }

  public LimelightReadings getTargetDataOdometry(Pose3d target) {
    LimelightReadings targetData = new LimelightReadings();

    Pose2d botPose = poseEstimator.getEstimatedPosition();
    Pose2d targetPose = target.toPose2d();

    //data from field positions
    double dx = targetPose.getX() - botPose.getX();
    double dy = targetPose.getY() - botPose.getY();
    double orthogonalAngle = Math.toDegrees(Math.atan2(dy, dx));
    //bot rotations at current pose
    double botPoseAngle = botPose.getRotation().getDegrees()%360;
    double angleOffset = botPoseAngle - orthogonalAngle;

    targetData.angleHorizontal = angleOffset; //degrees
    targetData.distance = Units.Meters.of(Math.hypot(dx, dy)); //meters
    targetData.angleVertical = 0;
    targetData.time = Timer.getFPGATimestamp();

    return targetData;
  }

  public void setPipeline(LimelightPipeline pipeline) {
    switch(pipeline) {
      case kNoVision:
      camera.getEntry("pipeline").setNumber(0);
      break;
      case kOdometry:
      camera.getEntry("pipeline").setNumber(1);
      break;
      case kZoom:
      camera.getEntry("pipeline").setNumber(2);
      break;
      case kSpeaker:
      camera.getEntry("pipeline").setNumber(3);
      break;
    }
  }

  //TODO: test this
  //Assuming path is linear
  public double getAngleToShooter(){
    var xToSpeaker = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue 
      ? poseEstimator.getEstimatedPosition().getX() 
      : 15.980 - poseEstimator.getEstimatedPosition().getX();
    var yToSpeaker = poseEstimator.getEstimatedPosition().getY();
    var distanceToSpeaker =
      Math.sqrt(
        Math.pow(xToSpeaker, 2) + Math.pow(yToSpeaker, 2)
      );
    var angle = Math.atan2(1.9, distanceToSpeaker);
    
    return angle;
  }
}