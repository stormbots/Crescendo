// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.reflect.Field;
import java.util.Optional;

import com.stormbots.Clamp;
import com.stormbots.LUT;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.FieldPosition;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.ShooterVision;

public class ShooterSetOdometry extends Command {
    private Shooter shooter;
    private ShooterFlywheel flywheel;
    Boolean exitsOnCompletion = true;
    double targetAngle = 0.0;
    double rpm = 0.0;
    SwerveDrivePoseEstimator pe;
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

    public ShooterSetOdometry(Shooter shooter, ShooterFlywheel flywheel, SwerveDrivePoseEstimator pe) {
        this.shooter = shooter;
        this.flywheel = flywheel;
        this.pe = pe;

        addRequirements(shooter);
        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double x = pe.getEstimatedPosition().getX()-FieldPosition.BlueSpeaker.getX();
        double y = pe.getEstimatedPosition().getY()-FieldPosition.BlueSpeaker.getY();
        double distance = Math.hypot(x, y);

        targetAngle = lut.get(distance)[0];
        rpm = lut.get(distance)[1];
        shooter.setAngle(targetAngle);
        flywheel.setRPM(rpm);
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

    public ShooterSetOdometry runForever(){
        this.exitsOnCompletion = false;
        return this;
    }
}