package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class DunkArmRoller extends SubsystemBase{
    public CANSparkMax rollerMotor = new CANSparkMax(Robot.isCompbot?16:15, MotorType.kBrushless);

    public DunkArmRoller() {
        rollerMotor.clearFaults();
        rollerMotor.restoreFactoryDefaults();

        rollerMotor.setSmartCurrentLimit(20);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("dunkArmRoller/output", rollerMotor.getAppliedOutput());
        SmartDashboard.putNumber("dunkArmRoller/current", rollerMotor.getOutputCurrent());
    }

    public void setSpeed(double speed) {
        rollerMotor.set(speed);
    }

    public void stop() {
        rollerMotor.set(0);
    }

    public void intake() {
        rollerMotor.set(0.1);
    }
    public void eject() {
        rollerMotor.set(-0.1);
    }

}
