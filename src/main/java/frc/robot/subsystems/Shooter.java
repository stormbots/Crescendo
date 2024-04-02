// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import java.lang.annotation.Target;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.stormbots.Clamp;
import com.stormbots.LUT;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public CANSparkMax shooterMotor = new CANSparkMax(Robot.isCompbot?14:13, MotorType.kBrushless);
  private SparkPIDController pidController = shooterMotor.getPIDController();
  private SparkAbsoluteEncoder  shooterAbsEncoder = shooterMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
  private double shooterSetPoint = 0.0;

  private double reverseSoftLimit = 2;
  private double forwardSoftLimit = 46;

  public boolean isHomed = false;
  public static double farthestShotDistance = 144; 

  public static LUT normalLUT = new LUT(new double[][]{
    {54, 44.4, 4000},
    {67.6, 39.19, 4000},
    {81.6, 33.31, 4000},
    {93.8, 29.42, 4000},
    {106, 27.56, 4000},
    {118.9, 24.12, 4000},
    {131.5, 20.97, 4500},
    {142.4, 19.44, 5500},
    {156.4, 18.63, 6000}, //last shot with dunkarm down
    {227, 9.132, 6000}
  });

  public static LUT constantShortLUT = new LUT(new double[][]{
    {54, 43.5, 5500},
    {63.2, 40.59, 5500},
    {73, 36, 5500},
    {84.6, 32.07, 5500},
    {96.5, 27.9, 5500},
    {107, 25.3, 5500},
    {118, 22.81, 5500},
    {130.4, 19.89, 5500},
    {142, 17.55, 5500}, 
    {144, 17.15, 5500}, //last shot with dunkarm down
    {153, 15.75, 5500}
  });

  public static LUT lobLut = new LUT(new double[][]{});

  public static final double kSlewForward = 150;
  public static final double kSlewBackward = -150;

  private final SysIdRoutine sysIdRoutine =
  new SysIdRoutine(
      // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
      new SysIdRoutine.Config(null, Units.Volts.of(4), null),//sorry for using nulls
      new SysIdRoutine.Mechanism(
          // Tell SysId how to plumb the driving voltage to the motors.
          (Measure<Voltage> volts) -> {
            shooterMotor.setVoltage(volts.in(Units.Volts));
          },
          log -> {
            log.motor("topMotor")
                .voltage(Units.Volts.of(shooterMotor.getAppliedOutput() * shooterMotor.getBusVoltage()))
                .linearPosition(Units.Meters.of(shooterMotor.getEncoder().getPosition()))
                .linearVelocity( Units.MetersPerSecond.of(shooterMotor.getEncoder().getVelocity()));
          },
          this));
          
  public Shooter() {
    shooterMotor.clearFaults();
    shooterMotor.restoreFactoryDefaults();

    shooterMotor.setClosedLoopRampRate(0.05);

    // pidController.setFeedbackDevice(shooterAbsEncoder);  //WARNING: This is potentially unsafe due to controlling through the discontinuity at 0; do not use. 
    pidController.setFeedbackDevice(shooterMotor.getEncoder()); //Make sure we revert to native encoder for PID

    //Configure Absolute encoder to accurate values
    shooterAbsEncoder.setPositionConversionFactor(360.0);
    shooterAbsEncoder.setInverted(false);
    shooterAbsEncoder.setVelocityConversionFactor(shooterAbsEncoder.getPositionConversionFactor()); //native unit is RPS

    //Configure relative encoder
    // shooterMotor.getEncoder().setPositionConversionFactor(45.0/11.51*0.955/2);//56.8/15.1 old 
    shooterMotor.getEncoder().setPositionConversionFactor((38.338787-3.184040)/(19.733170-2.209433));
    // shooterMotor.getEncoder().setPositionConversionFactor((41.827526-0.340941)/(21.474369-0.450663));
    shooterMotor.getEncoder().setVelocityConversionFactor(shooterMotor.getEncoder().getPositionConversionFactor()/60.0); //Native unit is RPM, so convert to RPS
    syncEncoders();
 
    reverseSoftLimit = getShooterAngle()+1;
    shooterMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) reverseSoftLimit);
    shooterMotor.setSoftLimit(SoftLimitDirection.kForward, (float) forwardSoftLimit);
    shooterMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    shooterMotor.enableSoftLimit(SoftLimitDirection.kForward, true);

    shooterMotor.setSmartCurrentLimit(20);

    //closed-loop control
    pidController.setP(6.0/1.2/360.0*1.7*1.1*1.5);
    pidController.setI(0.000000003*20);
    pidController.setD(0.00007*50);
    

    shooterMotor.setIdleMode(IdleMode.kCoast);

    shooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
    shooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
    shooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 10);
    shooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);

    shooterMotor.burnFlash();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("currenttesting/shooter", shooterMotor.getOutputCurrent());

    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("shooter/rotations", shooterMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("shooter/output", shooterMotor.getAppliedOutput());
    SmartDashboard.putNumber("shooter/absEncoder", getShooterAngleAbsolute());
    SmartDashboard.putNumber("shooter/encoder", shooterMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("shooter/target", shooterSetPoint);
    // SmartDashboard.putNumber("shooter/outputCurrent", shooterMotor.getOutputCurrent());
    // SmartDashboard.putNumber("shooter/TrapezoidProfile", getState().velocity);
    SmartDashboard.putBoolean("shooter/isOnTarget", isOnTarget());
    SmartDashboard.putNumber("shooter/angleDifference", shooterSetPoint-shooterMotor.getEncoder().getPosition());
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }
  
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  /** Align the absolute and relative encoders, should the need arise */
  public void syncEncoders(){
    var position = shooterAbsEncoder.getPosition();
    if(position > 225){
      //Account for discontinuity, set relative to negative position
      shooterMotor.getEncoder().setPosition(position-360);
    }else{
      shooterMotor.getEncoder().setPosition(position);
    }
  }

  public void setPower(double power){
    // shooterMotor.set(power);
  }

  public void stopShooter(){
    shooterMotor.set(0);
    PowerManager.getInstance().setPowerDraw(0, this);
  }
  
  public void moveShooter(double speed) {
    shooterMotor.set(speed + getShooterFFPercent());
  }

  public double getShooterAngle() {
    return shooterMotor.getEncoder().getPosition();
  }

  public double getShooterAngleAbsolute() {
    return shooterAbsEncoder.getPosition(); //in rotations, need to do limit
  }

  public boolean isOnTarget(){
    var tolerance = 0.75;
    //TODO figure out better tolerances that make sense
    return Clamp.bounded(shooterMotor.getEncoder().getPosition(), shooterSetPoint-tolerance, shooterSetPoint+tolerance);

  }
  public boolean isOnTarget(double position){
    var tolerance = 0.75;
    position = Clamp.clamp(position, reverseSoftLimit, forwardSoftLimit); 
    //TODO figure out better tolerances that make sense
    return Clamp.bounded(shooterMotor.getEncoder().getPosition(), position-tolerance, position+tolerance);
  }



  public double getShooterFFPercent(){
    var  kCosFFGain = 0.045;
    var ks = 0.0099 *Math.signum(this.shooterSetPoint-shooterMotor.getEncoder().getPosition());
    // 0.08 +- 0.09 according to tests
    return kCosFFGain*Math.cos(Math.toRadians(getShooterAngle())) + ks;
  }

  public void setAngle(double degrees) {
    degrees = Clamp.clamp(degrees, reverseSoftLimit, forwardSoftLimit); 
    this.shooterSetPoint = degrees;
    pidController.setReference(degrees, ControlType.kPosition, 0, getShooterFFPercent(),ArbFFUnits.kPercentOut);
  }

  public Command getDebugSetAngle(double degrees) {
    return new RunCommand(()->setAngle(degrees), this);
  }  

  public Command getManualMoveCommand(double speed){
    return new RunCommand(()->{moveShooter(speed);}, this)
    .finallyDo((end)->moveShooter(0));
  }

  public TrapezoidProfile.State getState(){
    // return new TrapezoidProfile.State(shooterAbsEncoder.getPosition(), shooterAbsEncoder.getVelocity());
    return new TrapezoidProfile.State(shooterMotor.getEncoder().getPosition(), shooterMotor.getEncoder().getVelocity());
  }

  public double getTargetAngle(){
    return shooterSetPoint;
  }
}
