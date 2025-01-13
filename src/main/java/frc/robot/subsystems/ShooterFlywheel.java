// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.stormbots.Clamp;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;

public class ShooterFlywheel extends SubsystemBase {

  public final static double kDunkArmTransferRPM = 600.0;
  public SparkFlex topMotor = new SparkFlex(Robot.isCompbot?12:11, MotorType.kBrushless);
  public SparkFlex botMotor = new SparkFlex(Robot.isCompbot?13:12, MotorType.kBrushless);

  private final double kGearing = 30/18.0;
  private final double kMaxRPM = 6784 * kGearing;

  double targetRPM = 0;
  
  public static final double kSlewForward = 4000/0.5;  //TODO: get rate limits
  public static final double kSlewBackward = -kSlewForward;

  SimpleMotorFeedforward topFlywheelFeedforward = new SimpleMotorFeedforward(0.13399, 0.0011067*0.955, 0.00019121);
  SimpleMotorFeedforward botFlywheelFeedforward = new SimpleMotorFeedforward(0.16668, 0.001071*0.98, 0.0001839);

   


  private final SysIdRoutine sysIdRoutine =
  new SysIdRoutine(
      // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          // Tell SysId how to plumb the driving voltage to the motors.
          (Voltage volts) -> {
            topMotor.setVoltage(volts.in(Units.Volts));
            botMotor.setVoltage(volts.in(Units.Volts));
          },
          log -> {
            log.motor("topMotor")
                .voltage(Units.Volts.of(topMotor.getAppliedOutput() * topMotor.getBusVoltage()))
                .linearPosition(Units.Meters.of(topMotor.getEncoder().getPosition()))
                .linearVelocity( Units.MetersPerSecond.of(topMotor.getEncoder().getVelocity()));
            log.motor("botMotor")
              .voltage(Units.Volts.of(botMotor.getAppliedOutput() * botMotor.getBusVoltage()))
              .linearPosition(Units.Meters.of(botMotor.getEncoder().getPosition()))
              .linearVelocity( Units.MetersPerSecond.of(botMotor.getEncoder().getVelocity()));
          },
          this));
          
  /** Creates a new Flywheel. */
  public ShooterFlywheel() {
    var topconfig = new SparkFlexConfig()
    .apply(Constants.kTypical)
    .inverted(false)
    .smartCurrentLimit(60)
    .idleMode(IdleMode.kCoast)
    ;
    topconfig.signals
    .primaryEncoderVelocityAlwaysOn(true)
    .primaryEncoderVelocityPeriodMs(5);
    topconfig.encoder
    .velocityConversionFactor(kGearing)
    .uvwMeasurementPeriod(8)
    .quadratureAverageDepth(2)
    .quadratureMeasurementPeriod(8)
    ;
    topconfig.closedLoop
    .velocityFF(1/kMaxRPM*10_000/9_111.0*(10_000/10_300.0))
    .p(0.000028853)
    .i(0.00007*45)
    .d(0.0000000003*3*5)
    .outputRange(-1,1)
    ;
    topconfig.closedLoop.maxMotion
    .maxVelocity(kMaxRPM)
    .maxAcceleration(kMaxRPM*4*4)
    ;
  
    var botconfig = new SparkFlexConfig()
    .apply(topconfig)
    .inverted(true)
    ;

    for(SparkFlex motor : new SparkFlex[]{topMotor,botMotor} ){
      motor.clearFaults();
      //disabled on currently running bot
      // motor.enableVoltageCompensation(10);
      //not found, but feels assumed
      // pid.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    }

    topMotor.configure(topconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    botMotor.configure(botconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {

    
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("/shooterFlywheel/targetRPM", targetRPM);
    SmartDashboard.putBoolean("/shooterFlywheel/isOnTarget", isOnTarget());
    SmartDashboard.putNumber("/shooterFlywheel/topCurrent", topMotor.getOutputCurrent());
    SmartDashboard.putNumber("/shooterFlywheel/botCurrent", botMotor.getOutputCurrent());
    SmartDashboard.putNumber("/shooterFlywheel/avgCurrentRPM", getRPM());
    SmartDashboard.putNumber("/shooterFlywheel/topMotorCurrentRPM", topMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("/shooterFlywheel/botMotorCurrentRPM", botMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("/shooterFlywheel/topMotorPercentOutput", topMotor.getAppliedOutput());
    SmartDashboard.putNumber("/shooterFlywheel/botMotorPercentOutput", botMotor.getAppliedOutput());

    // leftFlywheel.set(.1);
    
    // leftFlywheelPIDController.setReference(1200, SparkMax.ControlType.kVelocity);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }
  
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  public void setRPM(double targetRPM) {
    this.targetRPM = targetRPM;//Will need seprate target for right
    // topMotor.getPIDController().setReference(targetRPM, SparkMax.ControlType.kVelocity);
    // botMotor.getPIDController().setReference(targetRPM, SparkMax.ControlType.kVelocity);
    topMotor.getClosedLoopController().setReference(targetRPM, SparkMax.ControlType.kVelocity, ClosedLoopSlot.kSlot0, topFlywheelFeedforward.calculate(targetRPM), ArbFFUnits.kVoltage);
    botMotor.getClosedLoopController().setReference(targetRPM, SparkMax.ControlType.kVelocity, ClosedLoopSlot.kSlot0, botFlywheelFeedforward.calculate(targetRPM), ArbFFUnits.kVoltage);
  }

  public void setRPM(double topTargetRPM, double botTargetRPM) {
    
  }

  public void setRPMProfiled(double rpm){
    this.targetRPM = targetRPM;//Will need seprate target for right
    topMotor.getClosedLoopController().setReference(targetRPM, SparkMax.ControlType.kSmartVelocity);
    botMotor.getClosedLoopController().setReference(targetRPM, SparkMax.ControlType.kSmartVelocity);

  }

  public void stop(){
    topMotor.set(0.0);
    botMotor.set(0.0);
    
    PowerManager.getInstance().setPowerDraw(0, this);
  }

  public boolean isOnTarget(){
    double tolerance = 80;
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
    var config = new SparkMaxConfig().idleMode(mode);
    topMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    botMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public double getTargetRpm(){
    return targetRPM;
  }
}
