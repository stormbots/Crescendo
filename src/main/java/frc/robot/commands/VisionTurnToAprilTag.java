// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeVision;
import frc.robot.ChassisConstants;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.ShooterVision;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class VisionTurnToAprilTag extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private ShooterVision shooterVision;
  private Chassis chassis;
  private AHRS gyro;
  private DoubleSupplier xSpeed;
  private DoubleSupplier ySpeed;
  private DoubleSupplier rotSpeed;
  private double targetAngle = 0.0; //or 180?
  private double tolerance = 10.0;
  private double targetangle = 0.0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public VisionTurnToAprilTag(
    DoubleSupplier xSpeed,
    DoubleSupplier ySpeed,
    DoubleSupplier rotSpeed,
    ShooterVision shooterVision,
    Chassis chassis,
    AHRS gyro) {
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rotSpeed = rotSpeed;
    this.shooterVision = shooterVision;
    this.chassis = chassis;
    this.gyro = gyro;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterVision);
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterVision.setPipeline(ShooterVision.LimelightPipeline.kSpeaker);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //if valid target 
      // update target and bearing
    //if target is "newish", pid to it

    //in other cases, drive

    Optional<ShooterVision.LimelightReadings> shooterData = shooterVision.getVisibleTargetData();

    if (shooterData.isPresent()) {
      // targetangle = gyro.getRotation2d().getDegrees();
      // targetangle = targetangle - shooterData.get().angleHorizontal;
      // SmartDashboard.putNumber("shooterVision/targetAngle", shooterData.get().angleHorizontal);
      // SmartDashboard.putNumber("shooterVision/navxangle", gyro.getRotation2d().getDegrees());
      // targetangle = Math.toRadians(targetangle);
      // chassis.driveToBearing(xSpeed.getAsDouble(), ySpeed.getAsDouble(), targetangle);


      //doing it pure vision way, works!
      var tx = shooterData.get().angleHorizontal;
      tx = 0.1/360 * tx ;
      chassis.drive(xSpeed.getAsDouble(), ySpeed.getAsDouble(), rotSpeed.getAsDouble() -tx, true,true);
    }
    else{
      chassis.drive(xSpeed.getAsDouble(), ySpeed.getAsDouble(), rotSpeed.getAsDouble(), true,true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}