// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeVision;
import frc.robot.subsystems.IntakeVision.IntakePipeline;
import frc.robot.subsystems.LightEmittingDiodes;
import frc.robot.subsystems.Passthrough;
import frc.robot.subsystems.Chassis.Chassis;

public class VisionTrackNote extends Command {
  private IntakeVision intakeVision;
  private IntakePipeline pipeline;
  private Chassis chassis;
  private DoubleSupplier xSpeed;
  private DoubleSupplier ySpeed;
  private DoubleSupplier rotSpeed;
  private Intake intake;
  private Passthrough passthrough;
  private LightEmittingDiodes leds;

  /** Creates a new VisionTrackNoteOnHeading. */
  public VisionTrackNote(
    DoubleSupplier xSpeed, 
    DoubleSupplier ySpeed,
    DoubleSupplier rotSpeed,
    Chassis chassis,
    Intake intake,
    Passthrough passthrough, 
    IntakeVision intakeVision, 
    LightEmittingDiodes leds) {

    //TODO: put all subsystems last in constructor;


    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rotSpeed = rotSpeed;
    this.chassis = chassis;
    this.intake = intake;
    this.passthrough = passthrough;
    this.intakeVision = intakeVision;
    this.leds = leds;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
    addRequirements(intake);
    addRequirements(passthrough);
    addRequirements(intakeVision);
    addRequirements(leds);
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
    passthrough.setPower(0.1);

    var target = intakeVision.getVisibleTarget();
    var noteAdjustment = 0.0;

    if(target.isPresent()){
      leds.ready();
      noteAdjustment = -target.get().angleHorizontal * 1/360.0*5/2.0; //TODO roughly tuned PID borrowed from chassis.turnpid, but makes driver happy
    } else {
      leds.preparing();
    }

    chassis.drive(xSpeed.getAsDouble(), ySpeed.getAsDouble(), rotSpeed.getAsDouble() + noteAdjustment, true, true);
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
