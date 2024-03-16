// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterFlywheel;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class LogOutgoingShot extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private Shooter shooter;
    private ShooterFlywheel flywheel;
    SwerveDrivePoseEstimator pe;
    
    public LogOutgoingShot(SwerveDrivePoseEstimator pe, Shooter shooter, ShooterFlywheel flywheel) {
        this.shooter = shooter;
        this.flywheel = flywheel;
        this.pe = pe;
    }

    @Override
    public void initialize() {
        System.out.print("Shot Info - ");
        System.out.print(" X: " + pe.getEstimatedPosition().getX());
        System.out.print(" Y: " + pe.getEstimatedPosition().getY());
        System.out.print(" RPM: " + flywheel.getRPM());
        System.out.println(" Angle: " + shooter.getShooterAngle());
        System.out.println("Offset: " + RobotContainer.shooterOffset);
        System.out.print("Target Rpm" + flywheel.getTargetRpm());
        System.out.print(" Target Shooter Angle"  + shooter.getTargetAngle());
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
