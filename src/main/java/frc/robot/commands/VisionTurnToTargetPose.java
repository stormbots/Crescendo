package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FieldPosition;
import frc.robot.subsystems.FieldPosition.TargetType;
import frc.robot.subsystems.IntakeVision;
import frc.robot.subsystems.ShooterVision;
import frc.robot.subsystems.Chassis.Chassis;

public class VisionTurnToTargetPose extends Command {
  private IntakeVision intakeVision;
  private ShooterVision shooterVision;
  private Chassis chassis;
  private TargetType targetType;
  private Field2d field = new Field2d();

  public VisionTurnToTargetPose(
      TargetType targetType, ShooterVision shooterVision, Chassis chassis) {
    this.shooterVision = shooterVision;
    this.chassis = chassis;

    addRequirements(shooterVision);
    addRequirements(chassis);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // target readings + uploading
    Pose3d target = FieldPosition.getTargetList(targetType);
    field.getObject("targetpose").setPose(target.toPose2d());
    SmartDashboard.putData("targetposefield", field);

    // rotating to target
    ShooterVision.LimelightReadings shooterAngleToTarget =
        shooterVision.getTargetDataOdometry(target);
    double x =
        -shooterAngleToTarget
            .angleHorizontal; // angleHorizontal is angle offset from botpose angle to targetpose
    // angle
    var rotation = 0.5 / 60.0 * x;
    chassis.drive(0, 0, rotation, true, true);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
