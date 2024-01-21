// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Array;
import java.util.Optional;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BackVision extends SubsystemBase {
  /** Creates a new Vision. */
  private AHRS gyro;

  public enum LimelightPipeline {
    kNoZoom, kZoom
  }

  NetworkTable bp = NetworkTableInstance.getDefault().getTable("back limelight");
  
  NetworkTableEntry bpTable = bp.getEntry("botpose");
  //is there a way we can cross check the two bot poses????

  public double bpDefault [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  public Rotation2d rot = new Rotation2d(0,0);

  Field2d field = new Field2d();

  public BackVision(AHRS gyro) { //need to add pose estimator
    this.gyro = gyro;
    setPipeline(LimelightPipeline.kNoZoom);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double x = bp.getEntry("x").getDouble(0);
    double y = bp.getEntry("y").getDouble(0);

    double[] bp = bpTable.getDoubleArray(bpDefault);
    if (Array.getLength(bp)<6) {return;}

    rot = new Rotation2d( Math.toRadians(bp[5]) );

    //TODO: chassis stuff
  }

  public Optional<Double> getDistanceAprilTag() {
    //TODO: if has value return non-empty but no values rn :( (april tag)
    return Optional.empty();
  }

  public Optional<Double> getDistanceOdometry() {
    //TODO: if has value return non-empty but no values rn :( (odometry)
    return Optional.empty();
  }

  public Optional<Double> getAngleToNote() {
    //TODO: if has value return non-empty but no values rn :( (angle to note)
    return Optional.empty();
  }

  public void setPipeline(LimelightPipeline pipeline) {
    switch(pipeline) {
      case kNoZoom:
      bp.getEntry("pipeline").setNumber(0);
      break;
      case kZoom:
      bp.getEntry("pipeline").setNumber(1);
    }
  }
}
