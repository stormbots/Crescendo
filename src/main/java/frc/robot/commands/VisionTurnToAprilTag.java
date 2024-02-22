// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeVision;
import frc.robot.ChassisConstants;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.ShooterVision;

import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class VisionTurnToAprilTag extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private ShooterVision shooterVision;
  private IntakeVision intakeVision;
  private Chassis chassis;
  private AHRS gyro;
  private double targetAngle = 0.0; //or 180?
  private double tolerance = 10.0;
  private double angleError = 0.0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public VisionTurnToAprilTag(ShooterVision shooterVision, IntakeVision intakeVision, Chassis chassis, AHRS gyro) {
    this.shooterVision = shooterVision;
    this.intakeVision = intakeVision;
    this.chassis = chassis;
    this.gyro = gyro;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterVision);
    addRequirements(intakeVision);
    addRequirements(chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterVision.setPipeline(ShooterVision.LimelightPipeline.kNoZoom);
    intakeVision.setPipeline(IntakeVision.LimelightPipeline.kNoZoom);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<ShooterVision.LimelightReadings> shooterData = shooterVision.getVisibleTargetData();
    Optional<IntakeVision.LimelightReadings> intakeData = intakeVision.getVisibleTarget();

    if (shooterVision.hasValidTarget()) {
      angleError = shooterData.get().angleHorizontal;
      var continuousMin=-180;
      var continuousMax=180;
      if(continuousMin != continuousMax){
        var continuousHalfRange = (continuousMax-continuousMin)/2.0;
        angleError %= (continuousHalfRange*2);
        if(angleError>continuousHalfRange) angleError-=2*continuousHalfRange;
        if(angleError<-continuousHalfRange) angleError+=2*continuousHalfRange;
      }

      angleError= MathUtil.clamp(angleError, -5, 5);

      double turnOutput = angleError*0.5/60.0;
      turnOutput += 0.5/60.0*Math.signum(turnOutput);

      turnOutput += gyro.getRate() * .03/4.0; //.05 -> .03

      chassis.drive(0, 0, turnOutput, true, true);
    }
    // else
    // {
    //   //TODO: move VisionTurnToTargetPose here if it works
    // }

    else if (intakeVision.hasValidTarget()) {
      angleError = intakeData.get().angleHorizontal;
      var continuousMin=-180;
      var continuousMax=180;
      if(continuousMin != continuousMax){
        var continuousHalfRange = (continuousMax-continuousMin)/2.0;
        angleError %= (continuousHalfRange*2);
        if(angleError>continuousHalfRange) angleError-=2*continuousHalfRange;
        if(angleError<-continuousHalfRange) angleError+=2*continuousHalfRange;
      }

      angleError= MathUtil.clamp(angleError, -5, 5);

      double turnOutput = angleError*0.5/60.0;
      turnOutput += 0.5/60.0*Math.signum(turnOutput);

      turnOutput += gyro.getRate() * .03/4.0; //.05 -> .03

      chassis.drive(0, 0, turnOutput, true, true);
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