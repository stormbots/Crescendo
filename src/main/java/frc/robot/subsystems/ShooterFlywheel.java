// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.stormbots.Clamp;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterFlywheel extends SubsystemBase {
  public CANSparkMax topMotor = new CANSparkMax(11, MotorType.kBrushless);
  public CANSparkMax botMotor = new CANSparkMax(12, MotorType.kBrushless);

  private final double kGearing = 1.0;
  private final double kMaxRPM = 5760 * kGearing;

  double targetRPM = 0;
   
  /** Creates a new Flywheel. */
  public ShooterFlywheel() {

    for(CANSparkMax motor : new CANSparkMax[]{topMotor,botMotor} ){
      motor.restoreFactoryDefaults();
      motor.clearFaults();

      var pid = motor.getPIDController();
      pid.setP(0);
      pid.setFF(1/kMaxRPM);
      pid.setOutputRange(0,1); //dont know if we need this, adding just in case
      pid.setSmartMotionMaxVelocity(kMaxRPM, 0);
      pid.setSmartMotionMaxAccel(kMaxRPM/1.0, 0);
    }
    topMotor.setInverted(false);
    botMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("targetRPM", targetRPM);
    SmartDashboard.putBoolean("isOnTarget", isOnTarget());
    SmartDashboard.putNumber("currentRPM", getRPM());
    // leftFlywheel.set(.1);
    
    // leftFlywheelPIDController.setReference(1200, CANSparkMax.ControlType.kVelocity);
  }
  public void setRPM(double targetRPM) {
    this.targetRPM = targetRPM;//Will need seprate target for right
    topMotor.getPIDController().setReference(targetRPM, CANSparkMax.ControlType.kSmartVelocity);
    botMotor.getPIDController().setReference(targetRPM, CANSparkMax.ControlType.kSmartVelocity);
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
    return new RunCommand(()->setRPM(rpm) )
    .until(()->isOnTarget())
    ;
  }
}
