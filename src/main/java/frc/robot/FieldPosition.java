package frc.robot;

import java.util.ArrayList;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.IntakeVision;
import frc.robot.subsystems.IntakeVision.LimelightReadings;

public class FieldPosition {
    public final static double kOriginOffsetX=15.98/2.0;
    public final static double kOriginOffsetY=8.120/2.0;

    public final static double[] kSpeakerCols = {0.0, 0.0}; //TODO: get speaker cols
    public final static double kSpeakerRow = 0.0;
    public final static double kSpeakerHeight = 0; //TODO: get speaker height

    public final static Pose3d RedSpeaker= new Pose3d(kSpeakerCols[0], kSpeakerRow, kSpeakerHeight, new Rotation3d());
    public final static Pose3d BlueSpeaker= new Pose3d(kSpeakerCols[0], kSpeakerRow, kSpeakerHeight, new Rotation3d());

    /**
     * From https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf page 4
     * Distances are to center of field tag
     * */
    public final static ArrayList<Pose3d> AprilTags = new ArrayList<Pose3d>(){{
        //1, 2, : Blue source
        add( new Pose3d(Units.inchesToMeters(593.68), Units.inchesToMeters(9.68), Units.inchesToMeters(53.38), new Rotation3d(0,0, Math.toRadians(120))) );
        add( new Pose3d(Units.inchesToMeters(637.21), Units.inchesToMeters(34.79), Units.inchesToMeters(53.38), new Rotation3d(0,0,Math.toRadians(120))) );
        //3, 4 : Red speaker
        add( new Pose3d(Units.inchesToMeters(652.73), Units.inchesToMeters(196.17), Units.inchesToMeters(57.13), new Rotation3d(0,0,Math.PI)) );
        add( new Pose3d(Units.inchesToMeters(652.73), Units.inchesToMeters(218.42), Units.inchesToMeters(57.13), new Rotation3d(0,0,Math.PI)) );
        //5 : Red amp
        add( new Pose3d(Units.inchesToMeters(578.77), Units.inchesToMeters(323.00), Units.inchesToMeters(53.38), new Rotation3d(0,0,Math.toRadians(270))) );
        //6 : Blue amp
        add( new Pose3d(Units.inchesToMeters(72.5), Units.inchesToMeters(323.00), Units.inchesToMeters(53.38), new Rotation3d(0,0,Math.toRadians(270))) );
        //7, 8 : Blue speaker
        add( new Pose3d(Units.inchesToMeters(-1.50), Units.inchesToMeters(218.42), Units.inchesToMeters(57.13), new Rotation3d(0,0,0)) );
        add( new Pose3d(Units.inchesToMeters(-1.50), Units.inchesToMeters(196.17), Units.inchesToMeters(57.13), new Rotation3d(0,0,0)) );
        //9, 10 : Red source
        add( new Pose3d(Units.inchesToMeters(14.02), Units.inchesToMeters(34.79), Units.inchesToMeters(53.38), new Rotation3d(0,0,Math.toRadians(60))) );
        add( new Pose3d(Units.inchesToMeters(57.54), Units.inchesToMeters(9.68), Units.inchesToMeters(53.38), new Rotation3d(0,0,Math.toRadians(60))) );
        //11, 12, 13 : Red stage
        add( new Pose3d(Units.inchesToMeters(468.69), Units.inchesToMeters(146.19), Units.inchesToMeters(52.00), new Rotation3d(0,0,Math.toRadians(300))) );
        add( new Pose3d(Units.inchesToMeters(468.69), Units.inchesToMeters(177.10), Units.inchesToMeters(52.00), new Rotation3d(0,0,Math.toRadians(60))) );
        add( new Pose3d(Units.inchesToMeters(441.74), Units.inchesToMeters(161.62), Units.inchesToMeters(52.00), new Rotation3d(0,0,Math.PI)) );
        //14, 15, 16 : Blue stage
        add( new Pose3d(Units.inchesToMeters(209.48), Units.inchesToMeters(161.62), Units.inchesToMeters(52.00), new Rotation3d(0,0,0)) );
        add( new Pose3d(Units.inchesToMeters(182.73), Units.inchesToMeters(177.10), Units.inchesToMeters(52.00), new Rotation3d(0,0,Math.toRadians(120))) );
        add( new Pose3d(Units.inchesToMeters(182.73), Units.inchesToMeters(146.19), Units.inchesToMeters(52.00), new Rotation3d(0,0,Math.toRadians(240))) );
    }};

    ///////////////////////////
    //Utility Functions
    ///////////////////////////
    public static enum TargetType{Speaker}

    public static Pose3d GetTargetList(TargetType targetType){
      DriverStation.Alliance color = DriverStation.getAlliance().get();
      switch(targetType){
      case Speaker:   return color==Alliance.Red ? FieldPosition.RedSpeaker : FieldPosition.BlueSpeaker;
      }
      return new Pose3d();
    }

    public LimelightReadings getTargetDataOdometry(IntakeVision intakeVision, Pose3d target, SwerveDrivePoseEstimator poseEstimator) {
    LimelightReadings targetData = intakeVision.new LimelightReadings();

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
    targetData.distance = Math.hypot(dx, dy); //meters
    targetData.targetID = 0;
    targetData.angleVertical = 0;
    targetData.time = Timer.getFPGATimestamp();

    return targetData;
  }
    
    public static void ShowOnGlassDashboard(Field2d field){
        field.getObject("Red Speaker").setPoses(RedSpeaker.toPose2d());
        field.getObject("Blue Speaker").setPoses(BlueSpeaker.toPose2d());
    }
}