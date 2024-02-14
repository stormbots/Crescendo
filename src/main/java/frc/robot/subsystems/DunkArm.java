
package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.stormbots.Clamp;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DunkArm extends SubsystemBase {
  /** Creates a new Shooter. */
  public CANSparkMax armMotor = new CANSparkMax(14, MotorType.kBrushless);
  public CANSparkMax rollerMotor = new CANSparkMax(15, MotorType.kBrushless);
  private SparkPIDController armpid = armMotor.getPIDController();
  private SparkAbsoluteEncoder  armEncoder = armMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
  private double armSetpoint = 0.0;

  public DunkArm() {
    armMotor.restoreFactoryDefaults();
    armMotor.setIdleMode(IdleMode.kBrake);

    armpid.setP(0);


    rollerMotor.restoreFactoryDefaults();
    armMotor.setIdleMode(IdleMode.kBrake);

    //current limits?
    //soft limits
  }

  @Override
  public void periodic() {

    // shooterMotor.set(0.1);
    // shooterMotor.getPIDController().setReference(0, com.revrobotics.CANSparkBase.ControlType.kPosition);

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Rotations", armMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("output", armMotor.getAppliedOutput());
  }

  public void setIntake(double speed) {
    rollerMotor.set(speed);
  }

  public double getInternalEncoderAngle() {
    return armMotor.getEncoder().getPosition();
  }

  public double getShooterAngleAbsolute() {
    return armEncoder.getPosition(); //in rotations, need to do limit
  }

  public double isOnTarget(){
    //TODO figure out better tolerances that make sense
    return Clamp.clamp(armEncoder.getPosition(), armSetpoint-3, armSetpoint+3);
  }

  public void setArmAngle(double setPoint) {
    this.armSetpoint = setPoint;
    var armFF = Math.cos(Math.toRadians(getInternalEncoderAngle()));
    armpid.setReference(setPoint, com.revrobotics.CANSparkBase.ControlType.kPosition, 0, armFF,ArbFFUnits.kPercentOut); //TODO: voltage control
  }

  public Command getCommandMoveArmManually(double speed){
    return new RunCommand(()->{armMotor.set(speed);}, this)
    .finallyDo((end)->armMotor.set(0));
  }
}