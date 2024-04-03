// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.stormbots.Lerp;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.ShooterVision;

public class ShooterSetManually extends Command {
    private Shooter shooter;
    private ShooterFlywheel flywheel;
    private DoubleSupplier axis3;
    
    public ShooterSetManually(Shooter shooter, ShooterFlywheel flywheel, DoubleSupplier axis3) {
        SmartDashboard.putNumber("manualshoot/rpm",0);

        this.shooter = shooter;
        this.flywheel = flywheel;
        this.axis3 = axis3;

        addRequirements(shooter);
        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        flywheel.setRPM(SmartDashboard.getNumber("manualshoot/rpm", 0));
        shooter.setAngle(Lerp.lerp(axis3.getAsDouble(), -1, 1, 0, 45));
        SmartDashboard.putNumber("manualshoot/angle", shooter.getShooterAngle());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}