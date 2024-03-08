// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import com.stormbots.LUT;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.ShooterVision;

public class ShooterSetVision extends Command {
    private ShooterVision shooterVision;
    private Shooter shooter;
    private ShooterFlywheel flywheel;
    Boolean exitsOnCompletion = true;
    double targetAngle = 0.0;
    double rpm = 0.0;
    LUT lut = new LUT(new double[][]{
        {53.5, 40.0, 5400},
        {65.50, 30.0, 5500}, //5500
        {77.5, 31.0, 5600},
        {89.5, 30.0, 5700},
        {101.5, 24.0, 6500},
        {113.5, 20.0, 6500},
        {125.5, 19.0, 6500},
        {137.5, 16.0, 6500},
        {149.5, 17.0, 6750}
    });

    public ShooterSetVision(Shooter shooter, ShooterVision shooterVision, ShooterFlywheel flywheel) {
        this.shooter = shooter;
        this.shooterVision = shooterVision;
        this.flywheel = flywheel;
        addRequirements(shooter);
        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
        shooterVision.setPipeline(ShooterVision.LimelightPipeline.kSpeaker);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Optional<ShooterVision.LimelightReadings> visionData = shooterVision.getVisibleTargetData();
        if (visionData.isPresent()) {
            SmartDashboard.putBoolean("shootersetvision/running", true);
            double distance = -visionData.get().distance.in(Units.Inches);

            targetAngle = lut.get(distance)[0];
            rpm = lut.get(distance)[1];
            shooter.setAngle(targetAngle);
            flywheel.setRPM(rpm);
        }
        else {
            shooter.setAngle(targetAngle);
            flywheel.setRPM(rpm);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            shooter.setAngle(targetAngle);
            flywheel.setRPM(rpm);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return exitsOnCompletion && shooter.isOnTarget() && flywheel.isOnTarget();
    }

    public ShooterSetVision runForever(){
        this.exitsOnCompletion = false;
        return this;
    }
}