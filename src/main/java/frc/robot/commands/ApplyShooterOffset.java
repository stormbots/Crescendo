// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.stormbots.Lerp;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class ApplyShooterOffset extends Command {

    boolean wentDown = false;
    boolean wentUp = false;
    private DoubleSupplier axis3;

    public ApplyShooterOffset(DoubleSupplier axis3) {
        this.axis3 = axis3;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (axis3.getAsDouble() <= -0.95) wentDown = true;
        if (axis3.getAsDouble() >= 0.95) wentUp = true;
        if (wentUp && wentDown) {
            RobotContainer.shooterOffset = Lerp.lerp(
              axis3.getAsDouble(), 
              1, -1,
              RobotContainer.INITIALSHOOTEROFFSET-2, RobotContainer.INITIALSHOOTEROFFSET+5
            );
        }
        System.out.println("Shooter OffsetL" + RobotContainer.shooterOffset);
        System.out.println("up and down" + wentUp + wentDown);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        System.out.println("Shooter OffsetL" + RobotContainer.shooterOffset);
        System.out.println("up and down" + wentUp + wentDown);
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
