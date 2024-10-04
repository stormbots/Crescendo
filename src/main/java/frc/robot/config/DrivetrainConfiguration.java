// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Chassis.MAXSwerveModule;
import frc.robot.subsystems.Chassis.MAXSwerveModuleSparkMax;
import frc.robot.subsystems.Chassis.ModuleIO;


/** Add your docs here. */
public class DrivetrainConfiguration extends SubsystemBase{
    public double trackWidthIn; //x
    public double wheelBaseIn; //y
    public double wheelDiameterMeters;
    public int numModules;
    public SwerveModuleConfiguration[] modules;
    public double[] swerveModuleOffsets;
    public boolean isUsingSparkMax;
  
  public SwerveModuleConfiguration[] listModules (SwerveModuleConfiguration... swerveModules){
    for (var i = 0; i < numModules; i++){
      modules[i] = swerveModules[i];

    }
    return modules;
  }

  public double[] listModuleOffsets (SwerveModuleConfiguration... swerveModules){
    for (var i = 0; i < numModules; i++){
      swerveModuleOffsets[i] = swerveModules[i].getModuleOffset(swerveModules[i]);

    }
    return swerveModuleOffsets;
  }

  public ModuleIO[] getModules(DrivetrainConfiguration drivetrain){
    ModuleIO[] modules = new ModuleIO[drivetrain.numModules];
    if(drivetrain.isUsingSparkMax){
      for(var i =0; i < drivetrain.numModules; i++){
          modules[i] = new MAXSwerveModuleSparkMax(drivetrain.modules[i].drivingCanId, 
          drivetrain.modules[i].turningCanId, 
          drivetrain.modules[i].angularOffsetDegrees);
      }
  }
  else{
      for(var i =0; i < drivetrain.numModules; i++){
          modules[i] = new MAXSwerveModule(drivetrain.modules[i].drivingCanId, 
          drivetrain.modules[i].turningCanId, 
          drivetrain.modules[i].angularOffsetDegrees);
    }
  }
  return modules;
 }

  public DrivetrainConfiguration (double trackWidthIn, double wheelBaseIn, double wheelDiameterMeters, int numModules){
    this.trackWidthIn = trackWidthIn;
    this.wheelBaseIn = wheelBaseIn;
    this.wheelDiameterMeters = wheelDiameterMeters;
    this.numModules = numModules;
    }

    public class SwerveModuleConfiguration extends SubsystemBase{
        public int drivingCanId;
        public int turningCanId;
        public PIDController drivingPIDController;
        public PIDController turningPIDController;
        public double drivingFF;
        public double turningFF;
        public double angularOffsetDegrees;
        public double driveFreeSpeedRPM;
        public double drivingMotorPinionTeeth;
        public double drivingMotorSpurGearTeeth;
        private Translation2d getModulePosition(){
          double x = 0;
          double y = 0;

          double angularOffsetRadians = MathUtil.angleModulus(Math.toRadians(angularOffsetDegrees));
          
          if (angularOffsetRadians > 0.0 && angularOffsetRadians < (.5*Math.PI)){
            x = Math.tan(angularOffsetRadians)*Units.inchesToMeters(wheelBaseIn/2);
            y = Math.tan(angularOffsetRadians)*Units.inchesToMeters((trackWidthIn/2));
          }

          else if (angularOffsetRadians > (.5*Math.PI) && angularOffsetRadians < (Math.PI)){
            x = Math.tan(angularOffsetRadians)*Units.inchesToMeters(wheelBaseIn/2);
            y = (Math.tan(angularOffsetRadians)*Units.inchesToMeters((trackWidthIn/2)))*1;
          }

          else if (angularOffsetRadians > (Math.PI) && angularOffsetRadians < (1.5*Math.PI)){
            x = (Math.tan(angularOffsetRadians)*Units.inchesToMeters(wheelBaseIn/2))*-1;
            y = (Math.tan(angularOffsetRadians)*Units.inchesToMeters((trackWidthIn/2)))*1;
          }

          else if (angularOffsetRadians > (1.5*Math.PI) && angularOffsetRadians < (2*Math.PI)){
            x = (Math.tan(angularOffsetRadians)*Units.inchesToMeters(wheelBaseIn/2))*-1;
            y = Math.tan(angularOffsetRadians)*Units.inchesToMeters((trackWidthIn/2));
          }

          else if (angularOffsetRadians == 0.00 || angularOffsetRadians == (2*Math.PI)){
            y = Units.inchesToMeters((trackWidthIn/2));
          }

          else if (angularOffsetRadians == (.5*Math.PI)){
            x = Units.inchesToMeters((wheelBaseIn/2));
          }

          else if (angularOffsetRadians == (Math.PI)){
            y = Units.inchesToMeters((trackWidthIn/2));
          }

          else if (angularOffsetRadians == (1.5*Math.PI)){
            x = Units.inchesToMeters((wheelBaseIn/2));
          }

          Translation2d moduleLocation = new Translation2d(x,y);
          return moduleLocation;
        }

        public Translation2d modulePosition = getModulePosition();

        public double getModuleOffset(SwerveModuleConfiguration module){
          return module.angularOffsetDegrees;
        }

        public SwerveModuleConfiguration(int drivingCanId, int turningCanId, 
        PIDController drivingPIDController, PIDController turningPIDController, double drivingFF, double turningFF, 
        double angularOffsetDegrees, double driveFreeSpeedRPM, double drivingMotorPinionTeeth, 
        double drivingMotorSpurGearTeeth){

        }
        
      }
     
}
