// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import com.stormbots.LUT;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterVision;

public class ShooterSetVision extends Command {
    private ShooterVision shooterVision;
    private Shooter shooter;
    LUT lut = new LUT(new double[][]{
        {0.0, 0.0},
        {0.0, 0.0},
        {0.0, 0.0},
        {0.0,0.0},
        {0.0, 0.0}
    });
    
    public ShooterSetVision(Shooter shooter, ShooterVision shooterVision) {
        this.shooter = shooter;
        this.shooterVision = shooterVision;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Optional<ShooterVision.LimelightReadings> visionData = shooterVision.getVisibleTargetData();
        if (visionData.isEmpty()) {
            return;
        }

        double distance = visionData.get().distance;
        double shooterAngle = shooter.getShooterAngle();

        double target = lut.get(shooterAngle)[0];
        shooter.setAngle(target);
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