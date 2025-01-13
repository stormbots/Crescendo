// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import com.stormbots.LUT;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FieldPosition;
import frc.robot.subsystems.FieldPosition.TargetType;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterFlywheel;

public class ShooterSetOdometry extends Command {
    private Shooter shooter;
    private ShooterFlywheel flywheel;
    Boolean exitsOnCompletion = true;
    double targetAngle = 0.0;
    double rpm = 0.0;
    SwerveDrivePoseEstimator pe;
    Optional<Pose2d> manualPose = Optional.empty();

    LUT lut = Shooter.constantShortLUT;
    double x = 0.0;
    double y = 0.0;
    double distance = 0;

    public ShooterSetOdometry(Shooter shooter, ShooterFlywheel flywheel, SwerveDrivePoseEstimator pe) {
        this.shooter = shooter;
        this.flywheel = flywheel;
        this.pe = pe;

        addRequirements(shooter);
        addRequirements(flywheel);
    }

    public ShooterSetOdometry(Shooter shooter, ShooterFlywheel flywheel, Pose2d pose) {
        this.shooter = shooter;
        this.flywheel = flywheel;
        this.manualPose = Optional.of(pose);

        addRequirements(shooter);
        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        var pose = manualPose.orElse(pe.getEstimatedPosition());
        
        x = Units.Meters.of(Math.abs(pose.getX()-FieldPosition.getTargetList(TargetType.Speaker).getX())).in(Units.Inches);
        y = Units.Meters.of(Math.abs(pose.getY()-FieldPosition.getTargetList(TargetType.Speaker).getY())).in(Units.Inches);
        distance = Math.hypot(x, y);
        
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