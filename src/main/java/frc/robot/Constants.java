// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Constants {
    public static final Mode currentMode = Mode.real;

    public static enum Mode {
      /** Running on a real robot. */
      real,
  
      /** Running a physics simulator. */
      sim,
  
      /** Replaying from a log file. */
      replay
    } 
}
