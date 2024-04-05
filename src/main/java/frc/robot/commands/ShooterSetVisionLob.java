// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import com.stormbots.LUT;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FieldPosition;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.ShooterVision;

public class ShooterSetVisionLob extends Command {
    private ShooterVision shooterVision;
    private Shooter shooter;
    private ShooterFlywheel flywheel;
    Boolean exitsOnCompletion = true;
    double targetAngle = 10;
    double targetRPM = 4000;
    double targetAngleSlew = 0.0;
    double targetRPMSlew = 0.0;
    LUT lut = Shooter.lobLUT;
    SlewRateLimiter shooterRateLimiter =new SlewRateLimiter(
        Shooter.kSlewForward, Shooter.kSlewBackward, 0); //TODO: get rate limits
    SlewRateLimiter flywheelRateLimiter = new SlewRateLimiter(
        ShooterFlywheel.kSlewForward, ShooterFlywheel.kSlewBackward, 0); //TODO: get rate limits
    Pose3d lobTarget = new Pose3d();

    public ShooterSetVisionLob(Shooter shooter, ShooterVision shooterVision, ShooterFlywheel flywheel) {
        this.shooter = shooter;
        this.shooterVision = shooterVision;
        this.flywheel = flywheel;

        addRequirements(shooter);
        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
        Alliance color = DriverStation.getAlliance().orElse(Alliance.Blue);
        lobTarget = color==Alliance.Red ? FieldPosition.RedLob : FieldPosition.BlueLob;
        shooterRateLimiter.reset(shooter.getShooterAngle());
        flywheelRateLimiter.reset(flywheel.getRPM());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        var visionData = shooterVision.getTargetDataOdometry(lobTarget);
        if (visionData.isPresent()) {
            double distance = visionData.get().distance.in(Units.Inches);

            targetAngle = lut.get(distance)[0]; //get lut
            targetRPM = lut.get(distance)[1];

            targetAngleSlew = shooterRateLimiter.calculate(targetAngle); //set shooter slew
            shooter.setAngle(targetAngleSlew);

            targetRPMSlew = flywheelRateLimiter.calculate(targetRPM); //set flywheel slew
            flywheel.setRPM(targetRPMSlew);

        }
        else {
            targetAngleSlew = shooterRateLimiter.calculate(targetAngle); //set shooter slew
            shooter.setAngle(targetAngleSlew);
            targetRPMSlew = flywheelRateLimiter.calculate(targetRPM);
            flywheel.setRPM(targetRPMSlew);
        }
        

        SmartDashboard.putNumber("targetVisionRpm", targetRPM);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            targetAngleSlew = shooterRateLimiter.calculate(targetAngle); //set shooter slew
            shooter.setAngle(targetAngleSlew);
            targetRPMSlew = flywheelRateLimiter.calculate(targetRPM);
            flywheel.setRPM(targetRPMSlew);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return exitsOnCompletion && shooter.isOnTarget() && flywheel.isOnTarget();
    }

    public ShooterSetVisionLob runForever(){
        this.exitsOnCompletion = false;
        return this;
    }
}