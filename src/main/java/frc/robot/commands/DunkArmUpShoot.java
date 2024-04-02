// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.stormbots.LUT;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DunkArm;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.ShooterVision;
import java.util.Optional;

public class DunkArmUpShoot extends Command {
  DunkArm dunkArm;
  Shooter shooter;
  ShooterFlywheel flywheel;
  ShooterVision shooterVision;
  LUT lut = Shooter.lut;
  double targetRPM = 0.0;
  double targetAngle = 0.0;
  double lowestAngle = 13.0;
  boolean exitsOnCompletion = true;

  /** Creates a new DunkArmRollerHoldNote. */
  public DunkArmUpShoot(
      DunkArm dunkArm, Shooter shooter, ShooterFlywheel flywheel, ShooterVision shooterVision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.dunkArm = dunkArm;
    this.shooter = shooter;
    this.flywheel = flywheel;
    this.shooterVision = shooterVision;

    addRequirements(dunkArm);
    addRequirements(shooter);
    addRequirements(flywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterVision.setPipeline(ShooterVision.LimelightPipeline.kSpeaker);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<ShooterVision.LimelightReadings> visionData = shooterVision.getVisibleTargetData();
    if (visionData.isPresent()) {
      double distance = -visionData.get().distance.in(Units.Inches);

      targetAngle = lut.get(distance)[0];
      targetRPM = lut.get(distance)[1];
      shooter.setAngle(targetAngle);
      flywheel.setRPM(targetRPM);

      if (targetAngle <= lowestAngle) { // or do distance
        dunkArm.setArmAngle(105);
      }
    } else {
      shooter.setAngle(targetAngle);
      flywheel.setRPM(targetRPM);

      if (targetAngle <= lowestAngle) { // or do distance
        dunkArm.setArmAngle(105);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      shooter.setAngle(targetAngle);
      flywheel.setRPM(targetRPM);
      if (targetAngle <= lowestAngle) { // or do distance
        dunkArm.setArmAngle(105);
      }
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return exitsOnCompletion && shooter.isOnTarget() && flywheel.isOnTarget();
  }

  public DunkArmUpShoot runForever() {
    this.exitsOnCompletion = false;
    return this;
  }
}
