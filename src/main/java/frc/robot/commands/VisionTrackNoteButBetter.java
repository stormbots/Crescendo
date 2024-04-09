// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeVision;
import frc.robot.subsystems.IntakeVision.IntakePipeline;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Passthrough;

public class VisionTrackNoteButBetter extends Command {
  private IntakeVision intakeVision;
  private Chassis chassis;
  private Intake intake;
  private Passthrough passthrough;
  private Leds leds;
  private RobotContainer rc;

  /** Creates a new VisionTrackNoteOnHeading. */
  public VisionTrackNoteButBetter(RobotContainer rc) {

    //TODO: put all subsystems last in constructor;

    this.chassis = rc.chassis;
    this.intake = rc.intake;
    this.passthrough = rc.passthrough;
    this.intakeVision = rc.intakeVision;
    this.leds = rc.leds;
    this.rc = rc;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis, intake, passthrough, leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //set the vision pipeline
    intakeVision.setPipeline(IntakePipeline.kNote);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.intake();
    passthrough.intake();

    var target = intakeVision.getVisibleTarget();
    var noteAdjustment = 0.0;

    if(target.isPresent()){
      leds.ready();
      noteAdjustment = -target.get().angleHorizontal * 1/360.0*5/2.0*5.4; //TODO roughly tuned PID borrowed from chassis.turnpid, but makes driver happy
    } else {
      leds.preparing();
    }

    double scalar = 3;//If we would ever like to pid to the note rather than stop immediately

    SwerveModuleState[] desiredModuleStates = rc.swerveDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0, scalar, noteAdjustment));
    
    chassis.setModuleStates(desiredModuleStates);

  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    passthrough.stop();

    //intakeVision.setPipeline(IntakePipeline.kDriverView);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //check lasercan to see if it sees a note, then end
    return intake.isBlocked();
  }
}
