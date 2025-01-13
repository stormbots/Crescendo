package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class DunkArmRoller extends SubsystemBase{
    private SparkMax rollerMotor = new SparkMax(Robot.isCompbot?16:15, MotorType.kBrushless);

    SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.03, 0);

    public DunkArmRoller() {
        var config = new SparkMaxConfig()
        .smartCurrentLimit(10)
        ;
        var pcf = 1.375*Math.PI/3.0;
        config.encoder
        .positionConversionFactor(pcf)
        .velocityConversionFactor(pcf/60.0)
        ;
        config.closedLoop
        .p(0.05)
        ;
        config.softLimit
        .forwardSoftLimit(3.2).forwardSoftLimitEnabled(false)
        .reverseSoftLimit(-4.5).forwardSoftLimitEnabled(false)
        ;

        rollerMotor.clearFaults();

        rollerMotor.setCANTimeout(0);

        config.apply(Constants.kTypical);
        rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("dunkArmRoller/output", rollerMotor.getAppliedOutput());
        SmartDashboard.putNumber("currenttesting/dunkArmRoller", rollerMotor.getOutputCurrent());
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
        rollerMotor.set(-0.7);
    }

    public void scoreTrap(){
        setSpeed(-0.1-0.05);
    }

    public void scoreAmp(){
        setSpeed(-0.3);
    }

    public void resetEncoder(double encoderValue){
        rollerMotor.getEncoder().setPosition(encoderValue);
    }

    public void setPosition(double position){
        var delta = position - rollerMotor.getEncoder().getPosition();
        var feedforward = ff.calculate(delta);
        rollerMotor.getClosedLoopController().setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedforward, ArbFFUnits.kPercentOut );
    }

    public double getPosition(){
        return rollerMotor.getEncoder().getPosition();
    }

    public double getVelocity(){
        return rollerMotor.getEncoder().getVelocity();

    }

    public void setIdleMode(IdleMode mode){
        var config = new SparkMaxConfig().idleMode(mode);
        rollerMotor.configureAsync(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }
    public void enableSoftLimit(boolean enable){
        var config = new SparkMaxConfig();
        config.softLimit
        .reverseSoftLimitEnabled(enable)
        .forwardSoftLimitEnabled(enable)
        ;
        rollerMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    }

}
