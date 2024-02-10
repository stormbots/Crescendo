
package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.stormbots.Clamp;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DunkArm extends SubsystemBase {
  /** Creates a new Shooter. */
  public CANSparkMax armMotor = new CANSparkMax(14, MotorType.kBrushless);
  private SparkPIDController armPID = armMotor.getPIDController();
  private SparkAbsoluteEncoder armAbsEncoder = armMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
  private double armSetpoint = 0.0;

  public DunkArm() {
    armMotor.clearFaults();
    armMotor.restoreFactoryDefaults();
    armMotor.setIdleMode(IdleMode.kBrake);
    armAbsEncoder.setPositionConversionFactor(360.0);
    armMotor.getPIDController().setFeedbackDevice(armMotor.getEncoder()); //Make sure we revert to native encoder for PID
    armPID.setP(1/100.0);
    armMotor.getEncoder().setPositionConversionFactor(21.8/3.0);
    armAbsEncoder.setInverted(true);
    armMotor.getEncoder().setVelocityConversionFactor(armMotor.getEncoder().getPositionConversionFactor()); //native unit is RPS
    //current limits?
    //soft limits

    syncEncoders();

    armMotor.setSoftLimit(SoftLimitDirection.kReverse, -10);
    armMotor.setSoftLimit(SoftLimitDirection.kForward,104);
    armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    armMotor.setSmartCurrentLimit(20);
  }

  @Override
  public void periodic() {

    // shooterMotor.set(0.1);
    // shooterMotor.getPIDController().setReference(0, com.revrobotics.CANSparkBase.ControlType.kPosition);

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("dunkArm/Absolute Encoder", armAbsEncoder.getPosition());
    SmartDashboard.putNumber("dunkArm/Rotations", armMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("dunkArm/output", armMotor.getAppliedOutput());
    SmartDashboard.putNumber("dunkArm/velocity", getArmState().velocity);

  }

  public void syncEncoders(){
    var position = armAbsEncoder.getPosition();
    if(position > 270){ //TODO:fix
      //Account for discontinuity, set relative to negative position
    armMotor.getEncoder().setPosition(position-360);
    }else{
      armMotor.getEncoder().setPosition(position);
    }
  }
  
  public void setArm(double speed) {
    armMotor.set(speed + getArmFFPercent());
  }

  public void stopArm() {
    armMotor.set(0);
  }

  public double getInternalEncoderAngle() {
    return armMotor.getEncoder().getPosition();
  }

  public double getShooterAngleAbsolute() {
    return armAbsEncoder.getPosition(); //in rotations, need to do limit
  }

  public double isOnTarget(){
    //TODO figure out better tolerances that make sense
    return Clamp.clamp(armAbsEncoder.getPosition(), armSetpoint-3, armSetpoint+3);
  }
  
  public double getArmFFPercent(){
    var  kCosFFGain = (0.025+0.055)/2.0;
    return kCosFFGain*Math.cos(Math.toRadians(getInternalEncoderAngle()));
  }

  public void setArmAngle(double degrees) {
    this.armSetpoint = degrees;
    Clamp.clamp(degrees, armMotor.getSoftLimit(SoftLimitDirection.kReverse), armMotor.getSoftLimit(SoftLimitDirection.kForward));
    armPID.setReference(degrees, com.revrobotics.CANSparkBase.ControlType.kPosition, 0, getArmFFPercent(),ArbFFUnits.kPercentOut); //TODO: voltage control
  }

  public TrapezoidProfile.State getArmState(){
    // return new TrapezoidProfile.State(shooterAbsEncoder.getPosition(), shooterAbsEncoder.getVelocity());
    return new TrapezoidProfile.State(armMotor.getEncoder().getPosition(), armMotor.getEncoder().getVelocity());
  }
}