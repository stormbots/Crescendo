// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Optional;

import com.stormbots.LUT;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DunkArm;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterFlywheel;
import frc.robot.subsystems.ShooterVision;

public class ShooterSetVisionDAUp extends Command {
    DunkArm dunkArm;
    Shooter shooter;
    ShooterFlywheel flywheel;
    ShooterVision shooterVision;

    LUT lut = Shooter.constantShortLUT;
    double targetRPM = 0.0;
    double targetAngle = 0.0;
    double targetArm = 105;
    double targetRPMSlew = 0.0;
    double targetAngleSlew = 0.0;
    double targetArmSlew = 0.0;

    boolean exitsOnCompletion = true;
    
    SlewRateLimiter shooterRateLimiter =new SlewRateLimiter(
        Shooter.kSlewForward, Shooter.kSlewBackward, 0); //TODO: get rate limits
    SlewRateLimiter flywheelRateLimiter = new SlewRateLimiter(
        ShooterFlywheel.kSlewForward, ShooterFlywheel.kSlewBackward, 0); //TODO: get rate limits
    SlewRateLimiter armRateLimiter =new SlewRateLimiter(
        DunkArm.forwardSlewRateLimit, DunkArm.reverseSlewRateLimit, -20);

    /** Creates a new DunkArmRollerHoldNote. */
    public ShooterSetVisionDAUp(DunkArm dunkArm, Shooter shooter, ShooterFlywheel flywheel, ShooterVision shooterVision) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.dunkArm = dunkArm; 
        this.shooter = shooter;
        this.flywheel = flywheel;
        this.shooterVision = shooterVision;

        addRequirements(dunkArm);
        addRequirements(shooter);
        addRequirements(flywheel);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        shooterVision.setPipeline(ShooterVision.LimelightPipeline.kSpeaker);
        shooterRateLimiter.reset(shooter.getShooterAngle());
        flywheelRateLimiter.reset(flywheel.getRPM());
        armRateLimiter.reset(dunkArm.getAngle());
        shooterVision.setLEDMode(2.0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        /**NOT TESTED!!!!!!!!!!!**/
        Optional<ShooterVision.LimelightReadings> visionData = shooterVision.getVisibleTargetData();
        if (visionData.isPresent()) {
            double distance = -visionData.get().distance.in(Units.Inches);

            targetAngle = lut.get(distance)[0];
            targetRPM = lut.get(distance)[1];

            targetAngleSlew = shooterRateLimiter.calculate(targetAngle); //set shooter slew
            shooter.setAngle(targetAngleSlew);

            targetRPMSlew = flywheelRateLimiter.calculate(targetRPM); //set flywheel slew
            flywheel.setRPM(targetRPMSlew);

            targetArmSlew = armRateLimiter.calculate(targetArm);
            dunkArm.setArmAngle(targetArmSlew);
        }
        else {
            targetAngleSlew = shooterRateLimiter.calculate(targetAngle); //set shooter slew
            shooter.setAngle(targetAngleSlew);
            targetRPMSlew = flywheelRateLimiter.calculate(targetRPM);
            flywheel.setRPM(targetRPMSlew);
            targetArmSlew = armRateLimiter.calculate(targetArm);
            dunkArm.setArmAngle(targetArmSlew);
        }
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (!interrupted) {
            targetAngleSlew = shooterRateLimiter.calculate(targetAngle); //set shooter slew
            shooter.setAngle(targetAngleSlew);

            targetRPMSlew = flywheelRateLimiter.calculate(targetRPM);
            flywheel.setRPM(targetRPMSlew);

            targetArmSlew = armRateLimiter.calculate(targetArm);
            dunkArm.setArmAngle(targetArmSlew);
        }
        shooterVision.setLEDMode(1.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return exitsOnCompletion && shooter.isOnTarget() && flywheel.isOnTarget() && dunkArm.isOnTarget();
    }

    public ShooterSetVisionDAUp runForever(){
        this.exitsOnCompletion = false;
        return this;
    }
}
