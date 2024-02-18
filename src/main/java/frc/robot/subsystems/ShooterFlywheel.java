// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController.AccelStrategy;
import com.stormbots.Clamp;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class ShooterFlywheel extends SubsystemBase {
  public CANSparkMax topMotor = new CANSparkMax(Robot.isCompbot?12:11, MotorType.kBrushless);
  public CANSparkMax botMotor = new CANSparkMax(Robot.isCompbot?13:12, MotorType.kBrushless);

  private final double kGearing = 2.0;
  private final double kMaxRPM = 6784 * kGearing;

  double targetRPM = 0;
   
  /** Creates a new Flywheel. */
  public ShooterFlywheel() {

    for(CANSparkMax motor : new CANSparkMax[]{topMotor,botMotor} ){
      motor.restoreFactoryDefaults();
      motor.clearFaults();
      motor.getEncoder().setVelocityConversionFactor(kGearing);

      motor.setSmartCurrentLimit(30);

      var pid = motor.getPIDController();
      pid.setP(0.0003);
      pid.setFF(1/kMaxRPM);
      pid.setOutputRange(-1,1); //dont know if we need this, adding just in case
      pid.setSmartMotionMaxVelocity(kMaxRPM, 0);
      pid.setSmartMotionMaxAccel(kMaxRPM*4*4, 0);
      pid.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    }
    topMotor.setInverted(false);
    botMotor.setInverted(true);

    setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {

    
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("/shooterFlywheel/targetRPM", targetRPM);
    // SmartDashboard.putBoolean("/shooterFlywheel/isOnTarget", isOnTarget());
    // SmartDashboard.putNumber("/shooterFlywheel/avgCurrentRPM", getRPM());
    // SmartDashboard.putNumber("/shooterFlywheel/topMotorCurrentRPM", topMotor.getEncoder().getVelocity());
    // SmartDashboard.putNumber("/shooterFlywheel/topMotorPercentOutput", topMotor.getAppliedOutput());

    // leftFlywheel.set(.1);
    
    // leftFlywheelPIDController.setReference(1200, CANSparkMax.ControlType.kVelocity);
  }
  public void setRPM(double targetRPM) {
    this.targetRPM = targetRPM;//Will need seprate target for right
    // topMotor.getPIDController().setReference(targetRPM, CANSparkMax.ControlType.kSmartVelocity);
    // botMotor.getPIDController().setReference(targetRPM, CANSparkMax.ControlType.kSmartVelocity);
    topMotor.getPIDController().setReference(targetRPM, CANSparkMax.ControlType.kVelocity);
    botMotor.getPIDController().setReference(targetRPM, CANSparkMax.ControlType.kVelocity);

  }

  public boolean isOnTarget(){
    double leftSetRPM = targetRPM;
    double tolerance = leftSetRPM * .05;
    //ADD RIGHT
    if ( ! Clamp.bounded(topMotor.getEncoder().getVelocity(), targetRPM-tolerance, targetRPM+tolerance)) return false;
    if ( ! Clamp.bounded(botMotor.getEncoder().getVelocity(), targetRPM-tolerance, targetRPM+tolerance)) return false;
    return true;
  }
  public double getRPM(){
    var rpm = topMotor.getEncoder().getVelocity();
    rpm += botMotor.getEncoder().getVelocity();
    rpm /= 2;
    return rpm;
  }

  public Command getShooterSetRPMCommand(double rpm){
    return new RunCommand(()->{
      setRPM(rpm) ;
      SmartDashboard.putNumber("flywheel/targetrpm",rpm);
    },this)
    // .until(()->isOnTarget())
    ;
  }

  public void setIdleMode(IdleMode mode){
    topMotor.setIdleMode(mode);
    botMotor.setIdleMode(mode);
  }
}
