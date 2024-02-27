package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.stormbots.LUT;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class DunkArmRoller extends SubsystemBase{
    private CANSparkMax rollerMotor = new CANSparkMax(Robot.isCompbot?16:15, MotorType.kBrushless);

    SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.04, 0);

    public DunkArmRoller() {
        rollerMotor.clearFaults();
        rollerMotor.restoreFactoryDefaults();

        rollerMotor.setSmartCurrentLimit(20);
        // rollerMotor.setIdleMode(IdleMode.kBrake);
        rollerMotor.getEncoder().setPositionConversionFactor(1.375*Math.PI/3.0);
        rollerMotor.getEncoder().setVelocityConversionFactor(rollerMotor.getEncoder().getPositionConversionFactor()/60.0);
        rollerMotor.getPIDController().setP(0.05);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("dunkArmRoller/output", rollerMotor.getAppliedOutput());
        SmartDashboard.putNumber("dunkArmRoller/current", rollerMotor.getOutputCurrent());
        SmartDashboard.putNumber("dunkArmRoller/position", rollerMotor.getEncoder().getPosition());
    }

    public void setSpeed(double speed) {
        rollerMotor.set(speed);
    }

    public void stop() {
        rollerMotor.set(0);
    }

    public void intake() {
        rollerMotor.set(0.4);
    }
    public void eject() {
        rollerMotor.set(-0.4);
    }

    public void scoreTrap(){
        setSpeed(-0.1);
    }

    public void resetEncoder(double encoderValue){
        rollerMotor.getEncoder().setPosition(encoderValue);
    }

    public void setPosition(double position){
        var delta = position - rollerMotor.getEncoder().getPosition();
        var feedforward = ff.calculate(delta);
        rollerMotor.getPIDController().setReference(position, ControlType.kPosition, 0, feedforward, ArbFFUnits.kPercentOut );
    }

    public double getPosition(){
        return rollerMotor.getEncoder().getPosition();
    }

    public double getVelocity(){
        return rollerMotor.getEncoder().getVelocity();

    }

}
