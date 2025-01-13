
package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.stormbots.Clamp;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;

public class DunkArm extends SubsystemBase {
  /** Creates a new Shooter. */
  public SparkMax armMotor = new SparkMax(Robot.isCompbot?15:14, MotorType.kBrushless);
  private SparkClosedLoopController armPID = armMotor.getClosedLoopController();
  private SparkAbsoluteEncoder armAbsEncoder = armMotor.getAbsoluteEncoder();
  private double reverseSoftLimit = -30;
  private double forwardSoftLimit = armMotor.configAccessor.softLimit.getForwardSoftLimit();
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
          (Voltage volts) -> {
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
    var armconfig = new SparkMaxConfig()
    .idleMode(IdleMode.kBrake)
    .closedLoopRampRate(0.05)
    .smartCurrentLimit(40)
    ;
    armconfig.absoluteEncoder
    .positionConversionFactor(360)
    .inverted(true)
    .velocityConversionFactor(360)
    ;
    armconfig.softLimit
    .forwardSoftLimit(120)
    .reverseSoftLimit(reverseSoftLimit)
    .forwardSoftLimitEnabled(true)
    .reverseSoftLimitEnabled(true)    
    ;
    armconfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(1/25.0)
    .outputRange(-0.2,0.2)
    ;
    var pcf=22.5/1.929;
    armconfig.encoder
    .positionConversionFactor(pcf)
    .velocityConversionFactor(pcf/60.0)
    ;

    armconfig.apply(Constants.kAbsEncoder);

    //Make sure to write before we try to sync things
    armMotor.configure(armconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    syncEncoders();
    Timer.delay(0.05);
    // reverseSoftLimit = getAngle() + 1;


    armSetpoint = reverseSoftLimit;
    setArmAngle(getAngle());
  }

  @Override
  public void periodic() {

    // shooterMotor.set(0.1);
    // shooterMotor.getPIDController().setReference(0, com.revrobotics.Spark.ControlType.kPosition);

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
    armPID.setReference(degrees, ControlType.kPosition, ClosedLoopSlot.kSlot0, getArmFFPercent(),ArbFFUnits.kPercentOut); //TODO: voltage control
  }

  public TrapezoidProfile.State getState(){
    return new TrapezoidProfile.State(armMotor.getEncoder().getPosition(), armMotor.getEncoder().getVelocity());
  }

  // public isOnTarget()
}