// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Clamp;

public class ShooterVision extends SubsystemBase {
  /** Creates a new Vision. */
  public enum LimelightPipeline {
    kNoVision, kOdometry, kZoom, kSpeaker, kAllTags
  }
  public class LimelightReadings {
    public Distance distance; //inches
    public double angleHorizontal; //degrees
    public double angleVertical; //degrees
    public double time;
  }

  public NetworkTable camera = NetworkTableInstance.getDefault().getTable("limelight-shooter");
  Field2d field = new Field2d();
  FieldPosition fieldPos = new FieldPosition();
  public SwerveDrivePoseEstimator poseEstimator;
  private double lasthHeartbeat=-1;
  private double lastOKHeartBeatTimer = 0.0;
  private boolean heartBeatOK = false;
  // private boolean autoVisionOdometryEnabled = false;
  private boolean autoVisionOdometryEnabled = true;
  private boolean printedHeartBeat = false;


  public ShooterVision(SwerveDrivePoseEstimator poseEstimator){ //need to add pose estimator
    this.poseEstimator = poseEstimator;

    setPipeline(LimelightPipeline.kSpeaker);

    field.getObject("Blue Lob").setPose(FieldPosition.BlueLob.toPose2d());
    field.getObject("Red Lob").setPose(FieldPosition.RedLob.toPose2d());
    // this.camera.getEntry("ledMode").setNumber(3);
  }

  @Override
  public void periodic() {
    if(DriverStation.isAutonomous() && !autoVisionOdometryEnabled){return;}
    //zoomIfPossible(); pipeline makes frames drop a lot
    updateOdometry();
  
    SmartDashboard.putData("shootervisionfield", field);
    SmartDashboard.putNumber("shootervision/tv", camera.getEntry("tv").getDouble(0.0));
    SmartDashboard.putBoolean("shooter/distanceinrange", distanceInRange());

    printHeartbeat();


    // if (hasValidTarget()) {SmartDashboard.putBoolean("shootervision/validtarget", true);}
    // var target = getVisibleTargetData();
    // if (target.isPresent()) {
    //   SmartDashboard.putNumber("manualshoot/distance", target.get().distance.in(Units.Inches));
    // }
  }

  public boolean hasValidTarget() {
    double tv = camera.getEntry("tv").getDouble(0.0);
    return (tv >= 1);
  }

  public Optional<LimelightReadings> getVisibleTargetData() {
    if (hasValidTarget()==false) {return Optional.empty();}

    double[] bp = camera.getEntry("targetpose_robotspace").getDoubleArray(new double[]{0,0,0,0,0,0});

    if (bp.length<6) {return Optional.empty();}
    SmartDashboard.putNumber("shootervision/testingdistance", Units.Inches.convertFrom(bp[2], Units.Meters));
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

    // double[] bp = camera.getEntry("botpose").getDoubleArray(new double[]{0,0,0,0,0,0});

    // if (bp.length<6) {return;}
    // Rotation2d rot = new Rotation2d(Math.toRadians(bp[5]));
    // Pose2d botPose = new Pose2d(bp[0]+15.980/2.0, bp[1]+8.210/2.0, rot);
    // poseEstimator.addVisionMeasurement(botPose, Timer.getFPGATimestamp());

    // var stdevs = new Matrix<>(Nat.N3(), Nat.N1(), new double[]{1,1,1}); //confidence checker
    // poseEstimator.setVisionMeasurementStdDevs(stdevs);

    // field.getRobotObject().setPose(poseEstimator.getEstimatedPosition());
    // field.getObject("shootervisionpose").setPose(botPose);

    camera.getEntry("robot_orientation_set").setDoubleArray(new double[]{poseEstimator.getEstimatedPosition().getRotation().getDegrees(),0.0,0.0,0.0,0.0,0.0});
    poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999)); //Megatag depends on gyro never changing/us constantly knowing it, default values
    var megaTagEntry = camera.getEntry("botpose_orb_wpiblue");//blue side origin, dont need to manage origin
    double[] bp = megaTagEntry.getDoubleArray(new double[]{0,0,0,0,0,0,0});

    if (bp.length<7) {return;}
    var timestamp = megaTagEntry.getLastChange()/1000000.0 - bp[6]/1000.0; //lastchange(microseconds) - latency(milliseconds) in seconds;
    var pose = new Pose2d(bp[0], bp[1], new Rotation2d(Units.Radians.convertFrom(bp[5], Units.Degrees)));
    if(true){//if we want to add in other checks
      poseEstimator.addVisionMeasurement(pose, timestamp);
    }
  }

  public Optional<Pose2d> getPoseDifference(Pose2d target) {
    if (target==null) return Optional.empty();

    Pose2d robotPose = poseEstimator.getEstimatedPosition();
    Pose2d deltaPose = target.relativeTo(robotPose);

    return Optional.of(deltaPose);
  }


  //NOTE: USE ABOVE FUNCTION, BELOW ONES ARE UNCONFIRMED
  /**
   * @return angle to the left/right as seen by robot
   */
  public Optional<Double> getOrthogonalAngleFR(Pose2d target) {
    if (target==null) return Optional.empty();

    Pose2d deltaPose = getPoseDifference(target).get();
    double orthognalAngle = deltaPose.getTranslation().getAngle().getDegrees();

    return Optional.of(orthognalAngle);
  }

  /**
   * @return the difference in their rotations relative to the field
   */
  public Optional<Double> getOrthogonalAngleRR(Pose2d target) {
    if (target==null) return Optional.empty();

    Pose2d deltaPose = getPoseDifference(target).get();
    double orthogonalAngle = deltaPose.getRotation().getDegrees();

    return Optional.of(orthogonalAngle);
  }

  public Optional<Double> getDistance(Pose2d target) {
    if (target==null) return Optional.empty();

    var deltaPose = getPoseDifference(target).get();
    double distance = deltaPose.getTranslation().getNorm();

    return Optional.of(distance);
  }

  private void printHeartbeat()  {
    var heartBeat = camera.getEntry("hb").getDouble(0.0);
    if ( heartBeat - lasthHeartbeat <1) {
      //all good
      lastOKHeartBeatTimer = Timer.getFPGATimestamp();
      heartBeatOK = true;
    }
    else {
      heartBeatOK = false;
    }

    if (Timer.getFPGATimestamp() - lastOKHeartBeatTimer > 0.5 && !printedHeartBeat){
      System.err.println("Limelight disconnect!");
      printedHeartBeat = true;
    }
    
    SmartDashboard.putBoolean("shootervision/heartbeat", heartBeatOK);
    lasthHeartbeat = heartBeat;
  }

  public void setPipeline(LimelightPipeline pipeline) {
    switch(pipeline) {
      case kNoVision:
      camera.getEntry("pipeline").setNumber(3);
      break;
      case kOdometry:
      camera.getEntry("pipeline").setNumber(3);
      break;
      case kZoom:
      camera.getEntry("pipeline").setNumber(3);
      break;
      case kSpeaker:
      camera.getEntry("pipeline").setNumber(3);
      break;
      case kAllTags:
      camera.getEntry("pipeline").setNumber(4);
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

  public void setLEDMode(double mode) {
    camera.getEntry("ledMode").setNumber(mode);
  }

  public boolean distanceInRange() {
    var target = getVisibleTargetData();
    if (target.isEmpty()) return false;
    if (-target.get().distance.in(Units.Inches)<=Shooter.farthestShotDistance) return true;
    else return false;
  }

  public void setIDFilter(double[] filteredIDS) {
    camera.getEntry("fiducial_id_filters_set").setDoubleArray(filteredIDS);
  }

  // public void selectAllAprilTags() {
  //   double[] idFilter = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
  //   setIDFilter(idFilter);
  // }

  // public void selectSpeakerAprilTags() {
  //   double[] idFilter = {4, 7};
  //   setIDFilter(idFilter);
  // }

  public void selectSpeakerPipeline() {
    setPipeline(LimelightPipeline.kSpeaker);
  }

  public void selectAllTagsPipeline() {
    setPipeline(LimelightPipeline.kAllTags);
  }

  public void enableAutoVision(boolean enabled){
    autoVisionOdometryEnabled = enabled;
  }

  public void takeSnapshot() {
    camera.getEntry("snapshot").setNumber(1);
  }

  public void resetSnapshot() {
    camera.getEntry("snapshot").setNumber(0);
  }

  public Field2d getField() {
    return field;
  }

    // public Optional<Double> getOrthogonalAngle(Pose3d target) {
  //   if (target==null) return Optional.empty();

  //   Pose2d botPose = poseEstimator.getEstimatedPosition();
  //   Pose2d targetPose = target.toPose2d();

  //   //data from field positions
  //   double dx = targetPose.getX() - botPose.getX();
  //   double dy = targetPose.getY() - botPose.getY();
  //   Double orthogonalAngle = Math.toDegrees(Math.atan2(dy, dx));
  //   return Optional.of(orthogonalAngle);
  // }

  // public Optional<Double> getOrthogonalAngle(Pose2d target) {
  //   if (target==null) return Optional.empty();
    
  //   Pose2d botPose = poseEstimator.getEstimatedPosition();

  //   //data from field positions
  //   double dx = target.getX() - botPose.getX();
  //   double dy = target.getY() - botPose.getY();
  //   Double orthogonalAngle = Math.toDegrees(Math.atan2(dy, dx));
  //   return Optional.of(orthogonalAngle);
  // }

  // public Optional<Double> getDistance(Pose2d target) {
  //   if (target==null) return Optional.empty();

  //   Pose2d botPose = poseEstimator.getEstimatedPosition();
  //   return Optional.of(botPose.getTranslation().getDistance(target.getTranslation()));
  // }

  //   public Optional<LimelightReadings> getTargetDataOdometry(Pose3d target) {
  //   if (target==null) return Optional.empty();
  //   LimelightReadings targetData = new LimelightReadings();

  //   Pose2d botPose = poseEstimator.getEstimatedPosition();
  //   Pose2d targetPose = target.toPose2d();

  //   //data from field positions
  //   double dx = targetPose.getX() - botPose.getX();
  //   double dy = targetPose.getY() - botPose.getY();
  //   double orthogonalAngle = Math.toDegrees(Math.atan2(dy, dx));
  //   //bot rotations at current pose
  //   double botPoseAngle = botPose.getRotation().getDegrees();
  //   double angleOffset = botPoseAngle - orthogonalAngle;
  //   angleOffset%=180;

  //   //do it this way next year, no copy paste :P
  //   // targetPose.getRotation().minus(botPose.getRotation());
  //   // targetPose.getTranslation().getDistance(other)

  //   targetData.angleHorizontal = angleOffset; //degrees
  //   targetData.distance = Units.Meters.of(Math.hypot(dx, dy)); //meters
  //   targetData.angleVertical = 0;
  //   targetData.time = Timer.getFPGATimestamp();

  //   // double distance = botPose.getTranslation().getDistance(targetPose.getTranslation());
  //   // double deltaAngle = botPose.getRotation().minus(targetPose.getRotation()).getDegrees();

  //   // targetData.distance = Units.Meters.of(distance);
  //   // targetData.angleVertical = 0;
  //   // targetData.time = Timer.getFPGATimestamp();
  //   // targetData.angleHorizontal = botPoseAngle +angleOffset;

  //   return Optional.of(targetData);
  // }

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

  //Could have made static, not applicable to all "default" cases, not well thought out, do not copy in future code this was a shortcut
  public LimelightReadings getDefaultLimelightReadings(){
    LimelightReadings defaultLimelightReadings = new LimelightReadings();
    defaultLimelightReadings.angleHorizontal = 9999;
    defaultLimelightReadings.angleVertical = 9999;
    defaultLimelightReadings.distance = Units.Inches.of(9999);
    defaultLimelightReadings.time = 9999;

    return defaultLimelightReadings;
  }
}