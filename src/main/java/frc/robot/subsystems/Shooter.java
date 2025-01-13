// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.stormbots.Clamp;
import com.stormbots.LUT;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  public SparkMax shooterMotor = new SparkMax(Robot.isCompbot?14:13, MotorType.kBrushless);
  private SparkClosedLoopController pidController = shooterMotor.getClosedLoopController();
  private SparkAbsoluteEncoder  shooterAbsEncoder = shooterMotor.getAbsoluteEncoder();
  private double shooterSetPoint = 0.0;

  private double reverseSoftLimit = 2;
  private double forwardSoftLimit = 46;

  public boolean isHomed = false;
  public static double farthestShotDistance = 167; 

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
    {54, 43.5+1, 5500},
    {63.2, 40.59, 5500},
    {73, 36, 5500},
    {84.6, 32.07, 5500},
    {96.5, 27.9, 5500},
    //WITH GRIP TAPE
    {108, 23.7, 5500},
    {119, 20.6, 5500},
    {128, 18.8, 5500},
    {139, 17.6, 5500},
    {147, 15.5, 5500},
    {157, 14.4, 5500},
    {167, 13.7, 5500} //approx limit
    //WITHOUT GRIP TAPE
    // {107, 25.3, 5500},
    // {118, 22.81, 5500},
    // {130.4, 19.89, 5500},
    // {142, 17.55, 5500}, 
    // {144, 17.15, 5500}, //last shot with dunkarm down
    // {153, 15.75, 5500}
  });

  public static LUT lobLUT = new LUT(new double[][]{
    {25*12, 32.3, 3650}, 
    {28*12, 32.3, 3750},
    {32*12, 32.3, 3750},
    {36*12, 35.6, 3750},
  });

  public static final double kSlewForward = 150;
  public static final double kSlewBackward = -150;

  private final SysIdRoutine sysIdRoutine =
  new SysIdRoutine(
      // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
      new SysIdRoutine.Config(null, Units.Volts.of(4), null),//sorry for using nulls
      new SysIdRoutine.Mechanism(
          // Tell SysId how to plumb the driving voltage to the motors.
          (Voltage volts) -> {
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

    var config = new SparkMaxConfig()
    .apply(Constants.kAbsEncoder)
    .closedLoopRampRate(0.05)
    .smartCurrentLimit(20)
    .idleMode(IdleMode.kCoast)
    ;
    config.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .p(6.0/1.2/360.0*1.7*1.1*1.5)
    .i(0.000000003*20)
    .d(0.00007*50)
    ;

    config.absoluteEncoder
    .positionConversionFactor(360)
    .velocityConversionFactor(360/60.0)
    .inverted(false)
    ;
    var pcf=(38.338787-3.184040)/(19.733170-2.209433);
    config.encoder
    .positionConversionFactor(pcf)
    .velocityConversionFactor(pcf/60.0)
    ;

    //We need to write configs out before the sync step, because we need to make 
    // certain we have our encoders properly configured before trying to read them
    shooterMotor.configure(config,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);

    //Configure relative encoder
    syncEncoders();
    Timer.delay(0.02);
    reverseSoftLimit = getShooterAngle()+1;

    var limitconfig=new SparkMaxConfig();
    limitconfig.softLimit
    .forwardSoftLimitEnabled(true)
    .reverseSoftLimitEnabled(true)
    .forwardSoftLimit(forwardSoftLimit)
    .reverseSoftLimit(reverseSoftLimit)
    ;
    shooterMotor.configure(limitconfig,ResetMode.kNoResetSafeParameters,PersistMode.kNoPersistParameters);
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
    var tolerance = 1;
    position = Clamp.clamp(position, reverseSoftLimit, forwardSoftLimit); 
    //TODO figure out better tolerances that make sense
    return Clamp.bounded(shooterMotor.getEncoder().getPosition(), position-tolerance, position+tolerance);
  }

  public boolean isReadyToIntake(){
    return shooterMotor.getEncoder().getPosition() < 20;
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
    pidController.setReference(degrees, ControlType.kPosition, ClosedLoopSlot.kSlot0, getShooterFFPercent(),ArbFFUnits.kPercentOut);
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
