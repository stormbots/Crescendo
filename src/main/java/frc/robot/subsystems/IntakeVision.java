// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeVision extends SubsystemBase {
  /** Creates a new Vision. */
  public enum IntakePipeline {
    kDriverView, kNote,
  }
  public class NoteData {
    public double distance; //meters
    public double angleHorizontal; //degrees
    public double angleVertical; //degrees
    public Double time;
  }

  public NetworkTable camera = NetworkTableInstance.getDefault().getTable("limelight-intake");

  public IntakeVision() {
    setPipeline(IntakePipeline.kNote);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("intakevision/tv", camera.getEntry("tv").getDouble(0.0));
  }

  private boolean hasValidTarget() {
    double tv = camera.getEntry("tv").getDouble(0.0);
    return tv >= 1;
  }

  public Optional<NoteData> getVisibleTarget() {
    if (hasValidTarget()==false) {return Optional.empty();}

    var target = new NoteData();
    target.distance = 0;
    target.angleHorizontal = camera.getEntry("tx").getDouble(0.0);
    target.angleVertical = camera.getEntry("ty").getDouble(0.0);
    target.time = Timer.getFPGATimestamp();

    return Optional.of(target);
  }

  public void setPipeline(IntakePipeline pipeline) {
    //TODO: returns a null
    switch(pipeline) {
      case kDriverView:
      camera.getEntry("pipeline").setNumber(0);
      break;
      case kNote:
      camera.getEntry("pipeline").setNumber(3);
      break;
      default:
    }
  }
}