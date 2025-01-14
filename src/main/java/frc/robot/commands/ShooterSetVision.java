// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import com.stormbots.Clamp;
import com.stormbots.LUT;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.ShooterVision;

public class ShooterSetVision extends Command {
    private ShooterVision shooterVision;
    private Shooter shooter;
    private ShooterFlywheel flywheel;
    Boolean exitsOnCompletion = true;
    Boolean fullRange = false;
    double targetAngle = 10;
    double targetRPM = 4000;
    double targetAngleSlew = 0.0;
    double targetRPMSlew = 0.0;
    LUT lut = Shooter.constantShortLUT;
    SlewRateLimiter shooterRateLimiter =new SlewRateLimiter(
        Shooter.kSlewForward, Shooter.kSlewBackward, 0); //TODO: get rate limits
    SlewRateLimiter flywheelRateLimiter = new SlewRateLimiter(
        ShooterFlywheel.kSlewForward, ShooterFlywheel.kSlewBackward, 0); //TODO: get rate limits

    public ShooterSetVision(Shooter shooter, ShooterVision shooterVision, ShooterFlywheel flywheel) {
        SmartDashboard.putNumber("shooter/offset", RobotContainer.shooterOffset);
        this.shooter = shooter;
        this.shooterVision = shooterVision;
        this.flywheel = flywheel;

        addRequirements(shooter);
        addRequirements(flywheel);
    }

    //Incorrectly overloaded to prevent issues
    public ShooterSetVision(Shooter shooter, ShooterVision shooterVision, ShooterFlywheel flywheel, double targetAngle, double targetRPM) {
        SmartDashboard.putNumber("shooter/offset", RobotContainer.shooterOffset);
        this.shooter = shooter;
        this.shooterVision = shooterVision;
        this.flywheel = flywheel;

        this.targetAngle=targetAngle;
        this.targetRPM=targetRPM;

        addRequirements(shooter);
        addRequirements(flywheel);
    }

    @Override
    public void initialize() {
        shooterVision.setPipeline(ShooterVision.LimelightPipeline.kSpeaker);
        shooterRateLimiter.reset(shooter.getShooterAngle());
        flywheelRateLimiter.reset(flywheel.getRPM());
        shooterVision.setLEDMode(3.0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putNumber("shooter/offset", RobotContainer.shooterOffset);
        Optional<ShooterVision.LimelightReadings> visionData = shooterVision.getVisibleTargetData();
        if (visionData.isPresent()) {
            double distance = -visionData.get().distance.in(Units.Inches);
            
            if (!fullRange) {
                distance = Clamp.clamp(distance, 0, Shooter.farthestShotDistance);
            }

            targetAngle = lut.get(distance)[0] + RobotContainer.shooterOffset; //get lut
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
        shooterVision.setLEDMode(1.0);
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

    public ShooterSetVision fullRange() {
        fullRange = true;
        return this;
    }
}