// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterVision;

public class VisionSetShooter extends Command {
    private ShooterVision shooterVision;
    private Shooter shooter;
    // private double visionDistanceFromShooter;
    // private double shooterHeightFromTarget;
    // private double shooterDistanceFromTarget;
    
    public VisionSetShooter(Shooter shooter, ShooterVision shooterVision) {
        this.shooter = shooter;
        this.shooterVision = shooterVision;
        addRequirements(shooter);
        addRequirements(shooterVision);
    }

    @Override
    public void initialize() {
        SmartDashboard.getNumber("vision/manualtargeting",0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // var visionData = shooterVision.getVisibleTargetData();

        // double distance = visionData.get().distance;

        // double angle = Math.atan2(shooterHeightFromTarget, shooterDistanceFromTarget);
        shooter.setAngle(SmartDashboard.getNumber("vision/manualtargeting",0)
        );
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