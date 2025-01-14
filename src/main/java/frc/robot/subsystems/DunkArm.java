
package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.stormbots.Clamp;
import com.stormbots.LUT;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;

public class DunkArm extends SubsystemBase {
  /** Creates a new Shooter. */
  public CANSparkMax armMotor = new CANSparkMax(Robot.isCompbot?15:14, MotorType.kBrushless);
  private SparkPIDController armPID = armMotor.getPIDController();
  private SparkAbsoluteEncoder armAbsEncoder = armMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
  private double reverseSoftLimit = -30;
  private double forwardSoftLimit = armMotor.getSoftLimit(SoftLimitDirection.kForward);
  public static double reverseSlewRateLimit = -90*1.7;
  public static double forwardSlewRateLimit = 90*1.7;
  private double armSetpoint = 0.0;

  ArmFeedforward armff = new ArmFeedforward(.015, .035, 0.13/45);  // (0.025+0.055)/2.0;

  private final SysIdRoutine sysIdRoutine =
  new SysIdRoutine(
      // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
      new SysIdRoutine.Config(null, Units.Volts.of(4), null),//sorry for using nulls
      new SysIdRoutine.Mechanism(
          // Tell SysId how to plumb the driving voltage to the motors.
          (Measure<Voltage> volts) -> {
            armMotor.setVoltage(volts.in(Units.Volts));
          },
          log -> {
            log.motor("dunkArm")
                .voltage(Units.Volts.of(armMotor.getAppliedOutput() * armMotor.getBusVoltage()))
                .linearPosition(Units.Meters.of(armMotor.getEncoder().getPosition()))
                .linearVelocity( Units.MetersPerSecond.of(armMotor.getEncoder().getVelocity()));
          },
          this));
          
  public DunkArm() {
    armMotor.clearFaults();
    armMotor.restoreFactoryDefaults();
    armMotor.setIdleMode(IdleMode.kBrake);

    //Configure abs encoder
    armAbsEncoder.setPositionConversionFactor(360.0);
    armAbsEncoder.setInverted(true);
    armAbsEncoder.setVelocityConversionFactor(armAbsEncoder.getPositionConversionFactor()); //native unit is RPS, degrees/second
    
    //configure relative encoder
    armMotor.getPIDController().setFeedbackDevice(armMotor.getEncoder()); //Make sure we revert to native encoder for PID
    armMotor.getEncoder().setPositionConversionFactor(22.5/1.929);//21.8/3.0
    armMotor.getEncoder().setVelocityConversionFactor(armMotor.getEncoder().getPositionConversionFactor()/60.0); //native unit is RPS
    
    armPID.setP((1/25.0));
    armMotor.setClosedLoopRampRate(0.05);
    armPID.setOutputRange(-0.2, 0.2);
    syncEncoders();
    Timer.delay(0.05);
    // reverseSoftLimit = getAngle() + 1;
    armMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) reverseSoftLimit);
    // armMotor.setSoftLimit(SoftLimitDirection.kReverse, -30);
    armMotor.setSoftLimit(SoftLimitDirection.kForward,120); //112 hardstop, 7.3 inch after note transfer to safep trap pos
    armMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    armMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    armMotor.setSmartCurrentLimit(40);

    armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
    armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
    
    armMotor.burnFlash();

    armSetpoint = reverseSoftLimit;
    setArmAngle(getAngle());
  }

  @Override
  public void periodic() {

    // shooterMotor.set(0.1);
    // shooterMotor.getPIDController().setReference(0, com.revrobotics.CANSparkBase.ControlType.kPosition);

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("dunkArm/Absolute Encoder", armAbsEncoder.getPosition());
    SmartDashboard.putNumber("dunkArm/Rotations", armMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("dunkArm/output", armMotor.getAppliedOutput());
    // SmartDashboard.putNumber("dunkArm/velocity", getState().velocity);
    SmartDashboard.putNumber("dunkArm/position", armMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("dunkArm/setpoint", armSetpoint);
    SmartDashboard.putNumber("currenttesting/dunkArm", armMotor.getOutputCurrent());
    SmartDashboard.putNumber("dunkArm/reverseSoftLimit", reverseSoftLimit);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }
  
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  public void syncEncoders(){
    var position = armAbsEncoder.getPosition();
    if(position > 270){
      //Account for discontinuity, set relative to negative position
    armMotor.getEncoder().setPosition(position-360);
    }else{
      armMotor.getEncoder().setPosition(position);
    }
  }
  
  public void setPower(double power) {
    armMotor.set(power);
  }

  public void setPowerFF(double power){
    setPower(power + getArmFFPercent());
  }
  
  public void stop() {
    armMotor.set(0);
    PowerManager.getInstance().setPowerDraw(0, this);
  }

  public double getAngle() {
    return armMotor.getEncoder().getPosition();
  }

  public double getAngleAbs() {
    return armAbsEncoder.getPosition(); //in rotations, need to do limit
  }

  public Boolean isOnTarget(){
    //TODO figure out better tolerances that make sense
    // return Clamp.clamp(armAbsEncoder.getPosition(), armSetpoint-3, armSetpoint+3);
    return Clamp.bounded(getAngle(), armSetpoint-3, armSetpoint+3);
  }
  
  public double getArmFFPercent(){
    var  kCosFFGain = 0.043;
    return kCosFFGain*Math.cos(Math.toRadians(getAngle()));
  }

  // public double getArmFFProperly(double position, double velocity){
  //   //TODO: Manage feed forward and implement f a new setArmAngle profiler
  //   var pos = Math.toRadians(position);
  //   var vel = Math.toRadians(velocity);
  //   return armff.calculate(pos, vel); //NOTE: must be in rad + rad/s
  // }

  public void setArmAngle(double degrees) {
    this.armSetpoint = degrees;
    degrees = Clamp.clamp(degrees, reverseSoftLimit, forwardSoftLimit);
    //SmartDashboard.putNumber("dunkArm/targetAngle", degrees);
    armPID.setReference(degrees, ControlType.kPosition, 0, getArmFFPercent(),ArbFFUnits.kPercentOut); //TODO: voltage control
  }

  public TrapezoidProfile.State getState(){
    return new TrapezoidProfile.State(armMotor.getEncoder().getPosition(), armMotor.getEncoder().getVelocity());
  }

  // public isOnTarget()
}