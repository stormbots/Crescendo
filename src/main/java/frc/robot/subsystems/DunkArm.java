
package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.stormbots.Clamp;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class DunkArm extends SubsystemBase {
  /** Creates a new Shooter. */
  public CANSparkMax armMotor = new CANSparkMax(Robot.isCompbot?15:14, MotorType.kBrushless);
  private SparkPIDController armPID = armMotor.getPIDController();
  private SparkAbsoluteEncoder armAbsEncoder = armMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
  private double armSetpoint = 0.0;

  ArmFeedforward armff = new ArmFeedforward(.015, .035, 0.13/45);  // (0.025+0.055)/2.0;


  public DunkArm() {
    armMotor.clearFaults();
    armMotor.restoreFactoryDefaults();
    armMotor.setIdleMode(IdleMode.kBrake);

    //Configure abs encoder
    armAbsEncoder.setPositionConversionFactor(360.0);
    armAbsEncoder.setInverted(true);
    armAbsEncoder.setVelocityConversionFactor(armAbsEncoder.getPositionConversionFactor()/60.0); //native unit is RPS, degrees/second
    
    //configure relative encoder
    armMotor.getPIDController().setFeedbackDevice(armMotor.getEncoder()); //Make sure we revert to native encoder for PID
    armMotor.getEncoder().setPositionConversionFactor(22.5/1.929);//21.8/3.0
    armMotor.getEncoder().setVelocityConversionFactor(armMotor.getEncoder().getPositionConversionFactor()/60.0); //native unit is RPS
    
    armPID.setP((1/100.0)*2*4/2);
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
    SmartDashboard.putNumber("dunkArm/position", getArmState().position);
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
  
  public void setPower(double power) {
    armMotor.set(power + getArmFFPercent());
  }

  public void stop() {
    armMotor.set(0);
  }

  public double getAngle() {
    return armMotor.getEncoder().getPosition();
  }

  public double getAngleAbs() {
    return armAbsEncoder.getPosition(); //in rotations, need to do limit
  }

  public double isOnTarget(){
    //TODO figure out better tolerances that make sense
    return Clamp.clamp(armAbsEncoder.getPosition(), armSetpoint-3, armSetpoint+3);
  }
  
  public double getArmFFPercent(){
    var  kCosFFGain = 0.043;
    return kCosFFGain*Math.cos(Math.toRadians(getAngle()));
  }

  public double getArmFFProperly(double position, double velocity){
    //TODO: Manage feed forward and implement in a new setArmAngle profiler
    var pos = Math.toRadians(position);
    var vel = Math.toRadians(velocity);
    return armff.calculate(pos, vel); //NOTE: must be in rad + rad/s
  }

  public void setArmAngle(double degrees) {
    this.armSetpoint = degrees;
    Clamp.clamp(degrees, armMotor.getSoftLimit(SoftLimitDirection.kReverse), armMotor.getSoftLimit(SoftLimitDirection.kForward));
    armPID.setReference(degrees, ControlType.kPosition, 0, getArmFFPercent(),ArbFFUnits.kPercentOut); //TODO: voltage control
  }

  public TrapezoidProfile.State getArmState(){
    return new TrapezoidProfile.State(armMotor.getEncoder().getPosition(), armMotor.getEncoder().getVelocity());
  }
}