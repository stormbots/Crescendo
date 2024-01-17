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

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private AHRS gyro;

  public enum LimelightPipeline {
    kNoZoom, kZoom
  }

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  
  // NetworkTableEntry tx = table.getEntry("tx"); 
  // NetworkTableEntry ty = table.getEntry("ty"); 
  // NetworkTableEntry ta = table.getEntry("ta"); 
  // NetworkTableEntry tv = table.getEntry("tv"); 
  NetworkTableEntry bpTable = table.getEntry("botpose"); //gets translation (x, y, z) and rotation (x, y, z) for bot pose; may or may not change

  public double bpDefault [] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  public Rotation2d rot = new Rotation2d(0,0);

  Field2d field = new Field2d();

  public Vision(AHRS gyro) { //need to add pose estimator
    this.gyro = gyro;
    setPipeline(LimelightPipeline.kNoZoom);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double x = SmartDashboard.getNumber("limelight/tx", 0);
    double y = SmartDashboard.getNumber("limelight/ty", 0);

    double[] bp = bpTable.getDoubleArray(bpDefault);
    if (Array.getLength(bp)<6) {return;}

    rot = new Rotation2d( Math.toRadians( bp[5]) );
  }

  public Optional<Double> getDistance() {
    //TODO: if has value return non-empty but no values rn :(
    return Optional.empty();
  }

  public void setPipeline(LimelightPipeline pipeline) {
    switch(pipeline) {
      case kNoZoom:
      table.getEntry("pipeline").setNumber(0);
      break;
      case kZoom:
      table.getEntry("pipeline").setNumber(1);
    }
  }
}
