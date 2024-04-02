// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkFlexFixes;
import com.revrobotics.SparkPIDController.AccelStrategy;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.stormbots.Clamp;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;

public class ShooterFlywheel extends SubsystemBase {
  public CANSparkFlex topMotor = new CANSparkFlex(Robot.isCompbot?12:11, MotorType.kBrushless);
  public CANSparkFlex botMotor = new CANSparkFlex(Robot.isCompbot?13:12, MotorType.kBrushless);

  private final double kGearing = 30/18.0;
  private final double kMaxRPM = 6784 * kGearing;

  double targetRPM = 0;
  
  public static final double kSlewForward = 4000/0.5;  //TODO: get rate limits
  public static final double kSlewBackward = -kSlewForward;

  SimpleMotorFeedforward topFlywheelFeedforward = new SimpleMotorFeedforward(0.14872, 0.001077, 0.0001627);
  SimpleMotorFeedforward botFlywheelFeedforward = new SimpleMotorFeedforward(0.1378, 0.001072, 0.00016379);

   


  private final SysIdRoutine sysIdRoutine =
  new SysIdRoutine(
      // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          // Tell SysId how to plumb the driving voltage to the motors.
          (Measure<Voltage> volts) -> {
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
    topMotor.restoreFactoryDefaults();
    botMotor.restoreFactoryDefaults();
    topMotor.setInverted(false);
    botMotor.setInverted(true);


    for(CANSparkFlex motor : new CANSparkFlex[]{topMotor,botMotor} ){
      motor.clearFaults();
      motor.getEncoder().setVelocityConversionFactor(kGearing);

      // motor.enableVoltageCompensation(10);

      motor.setSmartCurrentLimit(60);
      motor.getEncoder().setMeasurementPeriod(8);
      motor.getEncoder().setAverageDepth(2);
      SparkFlexFixes.setFlexEncoderAverageDepth(motor, 2);
      SparkFlexFixes.setFlexEncoderSampleDelta(motor, 8);

      var pid = motor.getPIDController();
      // pid.setFF(1/kMaxRPM*10_000/9_111.0*(10_000/10_300.0));
      // pid.setP(0.0003*1.5*2*0.9);
      pid.setP(0.000028853);
      // pid.setD(0.00007*45);//want about .11
      // pid.setI(0.0000000003*3*5);
      pid.setOutputRange(-1,1); //dont know if we need this, adding just in case
      pid.setSmartMotionMaxVelocity(kMaxRPM, 0);
      pid.setSmartMotionMaxAccel(kMaxRPM*4*4, 0);
      pid.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 200);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 1000);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);

    }

    setIdleMode(IdleMode.kCoast);

    
    topMotor.burnFlash();
    botMotor.burnFlash();
  }

  @Override
  public void periodic() {

    
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("/shooterFlywheel/targetRPM", targetRPM);
    // SmartDashboard.putBoolean("/shooterFlywheel/isOnTarget", isOnTarget());
    SmartDashboard.putNumber("/shooterFlywheel/topCurrent", topMotor.getOutputCurrent());
    SmartDashboard.putNumber("/shooterFlywheel/botCurrent", botMotor.getOutputCurrent());
    SmartDashboard.putNumber("/shooterFlywheel/avgCurrentRPM", getRPM());
    SmartDashboard.putNumber("/shooterFlywheel/topMotorCurrentRPM", topMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("/shooterFlywheel/botMotorCurrentRPM", botMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("/shooterFlywheel/topMotorPercentOutput", topMotor.getAppliedOutput());
    SmartDashboard.putNumber("/shooterFlywheel/botMotorPercentOutput", botMotor.getAppliedOutput());

    // leftFlywheel.set(.1);
    
    // leftFlywheelPIDController.setReference(1200, CANSparkMax.ControlType.kVelocity);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }
  
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  public void setRPM(double targetRPM) {
    this.targetRPM = targetRPM;//Will need seprate target for right
    // topMotor.getPIDController().setReference(targetRPM, CANSparkMax.ControlType.kVelocity);
    // botMotor.getPIDController().setReference(targetRPM, CANSparkMax.ControlType.kVelocity);
    topMotor.getPIDController().setReference(targetRPM, CANSparkMax.ControlType.kVelocity, 0, topFlywheelFeedforward.calculate(targetRPM), ArbFFUnits.kVoltage);
    botMotor.getPIDController().setReference(targetRPM, CANSparkMax.ControlType.kVelocity, 0, botFlywheelFeedforward.calculate(targetRPM), ArbFFUnits.kVoltage);
  }

  public void setRPM(double topTargetRPM, double botTargetRPM) {
    
  }

  public void setRPMProfiled(double rpm){
    this.targetRPM = targetRPM;//Will need seprate target for right
    topMotor.getPIDController().setReference(targetRPM, CANSparkMax.ControlType.kSmartVelocity);
    botMotor.getPIDController().setReference(targetRPM, CANSparkMax.ControlType.kSmartVelocity);

  }

  public void stop(){
    topMotor.set(0.0);
    botMotor.set(0.0);
    
    PowerManager.getInstance().setPowerDraw(0, this);
  }

  public boolean isOnTarget(){
    double tolerance = 60;
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

  public double getTargetRpm(){
    return targetRPM;
  }
}
