// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeVision;
import frc.robot.subsystems.IntakeVision.IntakePipeline;
import frc.robot.subsystems.Leds;
import frc.robot.subsystems.Passthrough;

public class VisionTrackNoteOnHeading extends Command {
  private IntakeVision intakeVision;
  private IntakePipeline pipeline;
  private Chassis chassis;
  private DoubleSupplier xSpeed;
  private DoubleSupplier ySpeed;
  private DoubleSupplier rotSpeed;
  private AHRS gyro;
  private Intake intake;
  private Passthrough passthrough;
  private Leds leds;
  private boolean isBlocked;

  /** Creates a new VisionTrackNoteOnHeading. */
  public VisionTrackNoteOnHeading(Chassis chassis, IntakeVision intakeVision, DoubleSupplier xSpeed, 
  DoubleSupplier ySpeed, DoubleSupplier rotSpeed, Intake intake, Passthrough passthrough, Leds leds) {

    //TODO: put all subsystems last in constructor;


    // this.vision = vision;
    // this.pipeline = pipeline;
    this.chassis = chassis;
    this.xSpeed = xSpeed;
    this.ySpeed = ySpeed;
    this.rotSpeed = rotSpeed;
    // this.gyro = gyro;
    this.intake = intake;
    this.passthrough = passthrough;
    this.leds = leds;
    this.intakeVision = intakeVision;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(chassis);
    addRequirements(intakeVision);
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //set the vision pipeline
    intakeVision.setPipeline(pipeline);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var target = intakeVision.getVisibleTarget();

    intake.intake();
    //passthrough?!
    var noteadjustment = 0.0;

    if(target.isPresent()){
      target.get();//todo use me!!!
      leds.ready();
      //add to noteoffset
      noteadjustment = intakeVision.angleOffset();
      // noteadjustment = target.getVisibleTarget();

    } else {
      leds.preparing();
    }

    chassis.drive(xSpeed.getAsDouble(), ySpeed.getAsDouble(), noteadjustment + rotSpeed.getAsDouble(), true, true);
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    //passthrough stop!?!

    intakeVision.setPipeline(IntakePipeline.kDriverView);
    leds.showTeamColor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //check lasercan to see if it sees a note, then end
    if (passthrough.isBlocked()) {
      return true;
    } else {
      return false;
    }
  }
}
