// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController.AccelStrategy;
import com.stormbots.Clamp;

import edu.wpi.first.units.Power;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class ShooterFlywheel extends SubsystemBase {
  public CANSparkFlex topMotor = new CANSparkFlex(Robot.isCompbot?12:11, MotorType.kBrushless);
  public CANSparkFlex botMotor = new CANSparkFlex(Robot.isCompbot?13:12, MotorType.kBrushless);

  private final double kGearing = 30/18.0;
  private final double kMaxRPM = 6784 * kGearing;

  double targetRPM = 0;
   
  /** Creates a new Flywheel. */
  public ShooterFlywheel() {

    for(CANSparkFlex motor : new CANSparkFlex[]{topMotor,botMotor} ){
      motor.restoreFactoryDefaults();
      motor.clearFaults();
      motor.getEncoder().setVelocityConversionFactor(kGearing);

      // motor.enableVoltageCompensation(10);

      motor.setSmartCurrentLimit(60);
      motor.getEncoder().setMeasurementPeriod(8);
      motor.getEncoder().setAverageDepth(2);

      var pid = motor.getPIDController();
      pid.setFF(1/kMaxRPM*10_000/9_111.0*(10_000/10_300.0));
      pid.setP(0.0003*1.5*2);
      pid.setD(0.00007*45);//want about .11
      pid.setI(0.0000000003*3*5);
      pid.setOutputRange(-1,1); //dont know if we need this, adding just in case
      pid.setSmartMotionMaxVelocity(kMaxRPM, 0);
      pid.setSmartMotionMaxAccel(kMaxRPM*4*4, 0);
      pid.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 200);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 200);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 200);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 1000);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 1000);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 1000);
      motor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 1000);

    }
    topMotor.setInverted(false);
    botMotor.setInverted(true);

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
  public void setRPM(double targetRPM) {
    this.targetRPM = targetRPM;//Will need seprate target for right
    // topMotor.getPIDController().setReference(targetRPM, CANSparkMax.ControlType.kSmartVelocity);
    // botMotor.getPIDController().setReference(targetRPM, CANSparkMax.ControlType.kSmartVelocity);
    topMotor.getPIDController().setReference(targetRPM, CANSparkMax.ControlType.kVelocity);
    botMotor.getPIDController().setReference(targetRPM, CANSparkMax.ControlType.kVelocity);

  }

  public void stop(){
    topMotor.set(0.0);
    botMotor.set(0.0);
    
    PowerManager.getInstance().setPowerDraw(0, this);
  }

  public boolean isOnTarget(){
    double leftSetRPM = targetRPM;
    double tolerance = leftSetRPM * .005;
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
    ;
  }

  public void setIdleMode(IdleMode mode){
    topMotor.setIdleMode(mode);
    botMotor.setIdleMode(mode);
  }
}
