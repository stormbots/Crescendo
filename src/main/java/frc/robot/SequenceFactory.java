// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.commands.FollowPathHolonomic;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ClimberSetPosition;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.NoteTransferToDunkArm;
import frc.robot.commands.PassthroughAlignNote;
import frc.robot.commands.SetDunkArmSlew;
import frc.robot.commands.SetShooterProfiled;
import frc.robot.subsystems.DunkArm;
import frc.robot.commands.ShooterSetVision;
import frc.robot.commands.VisionTurnToSpeakerOpticalOnly;
import frc.robot.subsystems.Shooter;

/** 
 * A place to keep/generate useful, reusable code sequences and commands.
 *  
 */
public class SequenceFactory {
    RobotContainer rc;

    public SequenceFactory(RobotContainer rc){
        this.rc = rc;
    }

    public Command ExampleSequence(){
        return new InstantCommand(()->{},rc.chassis,rc.climber);
    }

    public Command getDunkArmNoteTransferSequence(){
        return new ParallelCommandGroup(
            new SetDunkArmSlew(-25, rc.dunkArm),
            new SetShooterProfiled(0, rc.shooter),
            rc.shooterFlywheel.getShooterSetRPMCommand(500).until(()->rc.shooterFlywheel.isOnTarget())
        )
        .andThen(
            new ParallelCommandGroup(
                new RunCommand(()->rc.dunkArmRoller.setSpeed(0.1), rc.dunkArmRoller),
                new RunCommand(()->rc.passthrough.intake(), rc.passthrough),
                new RunCommand(()->rc.intake.intake(), rc.intake),
                rc.shooterFlywheel.getShooterSetRPMCommand(500)
            ).until(()->rc.passthrough.isBlocked()==false)
        )
        .andThen(
            new NoteTransferToDunkArm(rc.shooterFlywheel, rc.dunkArmRoller),
            new RunCommand(rc.passthrough::stop,rc.passthrough),
            new RunCommand(rc.intake::stop,rc.intake)
        );
    }

    public Command getSetRPMandShootCommand(double rpm, double angle){
        return new InstantCommand()
        .andThen(
            new ParallelCommandGroup(
                getStopIntakingCommand(),
                getToShooterStateCommand(rpm, angle)
            )
            .until(
                ()-> getShooterOnTarget(angle, rpm)
            )
            .withTimeout(2)
        )
        .andThen(
            new ParallelCommandGroup(
                new RunCommand(rc.passthrough::intake,rc.passthrough),
                new RunCommand(rc.intake::intake,rc.intake)
            )
            .until(()->!rc.passthrough.isBlocked())
            .withTimeout(0.5)
        )
        ;
    }

    //This one is sequenced slightly differently as we would like to turn while we are spinning up. Furthermore we dont want to require the chassis when we dont want to
    //Note that there might be issues with the the angles at edge cases, likely will never use it for that though
    public Command getTurnSetRPMandShootCommand(double targetBearing, double rpm, double shooterAngle){
        return new InstantCommand()
        .andThen(
            new ParallelCommandGroup(
                new RunCommand(rc.passthrough::stop, rc.passthrough),
                new RunCommand(()->rc.chassis.driveToBearing(Math.toRadians(targetBearing)), rc.chassis),
                rc.shooterFlywheel.getShooterSetRPMCommand(rpm),
                new SetShooterProfiled(shooterAngle, rc.shooter).runForever()
            )
            .until(
                ()-> getShooterOnTarget(shooterAngle, rpm) &&
                    Clamp.bounded(rc.navx.getRotation2d().getDegrees(), targetBearing-10, targetBearing+10) &&
                    Clamp.bounded(rc.navx.getRate(), -5, 5)
            )
            .withTimeout(2)
        )
        .andThen(
            new ParallelCommandGroup(
                new RunCommand(rc.passthrough::intake,rc.passthrough),
                new RunCommand(rc.intake::intake,rc.intake)
            )
            .until(()->!rc.passthrough.isBlocked())
            .withTimeout(0.5)
        )
        ;
    }

    public Command getStopShooterCommand(){
        return new ParallelCommandGroup(
            new SetShooterProfiled(0, rc.shooter),
            rc.shooterFlywheel.getShooterSetRPMCommand(0)
        )
        .until(()->getShooterOnTarget(0, 0))
        .withTimeout(0.5);
    }

    public Command getIntakeThenAlignCommand(){
        return new InstantCommand()
            .andThen(new ParallelDeadlineGroup(new IntakeNote(rc.intake, rc.passthrough), new RunCommand(()->rc.passthrough.lockServo(true))))
            .andThen(new ParallelDeadlineGroup(new PassthroughAlignNote(rc.passthrough, rc.intake), new WaitCommand(0.5).andThen(new RunCommand(()->rc.passthrough.lockServo(false))))
            );
    }

    public Command getTrapSequenceCommand(Command pathFollowingCommand, Measure<Angle> gyroAngle){
        return new InstantCommand()
            .andThen(new RunCommand(()->rc.chassis.driveToBearing(gyroAngle.in(Units.Radians)), rc.chassis).withTimeout(3))
            // .until(
            //     ()->Clamp.bounded(rc.navx.getRotation2d().getDegrees(), gyroAngle.in(Units.Degrees)-10, gyroAngle.in(Units.Degrees)+10) &&
            //     Clamp.bounded(rc.navx.getRate(), -5, 5)
            // )
            .andThen(
                new ParallelDeadlineGroup(
                    // new ClimberSetPosition(rc.climber, rc.climber.kMaxHeight),
                    pathFollowingCommand,
                    new RunCommand(()->rc.dunkArm.setArmAngle(
                        Clamp.clamp(Math.toDegrees(Math.atan2(0.2,  (0.8-rc.chassis.getDistanceFromStageCenter().in(Units.Meters))  )), //0.8
                        0, 110)), 
                        rc.dunkArm)                        
                )
                // .withTimeout(3.5)
            )
            .andThen(
                new ParallelCommandGroup(
                    // new ClimberSetPosition(rc.climber, Units.Inches.of(0)),
                    new SetDunkArmSlew(120, rc.dunkArm).runForever()
                )
            );
    }

    public Command getToShooterStateCommand(double rpm, double shooterAngle){
        return new ParallelCommandGroup(
            rc.shooterFlywheel.getShooterSetRPMCommand(rpm),
            new SetShooterProfiled(shooterAngle, rc.shooter).runForever());
    }

    public Command getStopIntakingCommand(){
        return new ParallelCommandGroup(
            new RunCommand(rc.passthrough::stop,rc.passthrough),
            new RunCommand(rc.intake::stop,rc.intake));
    }

    public boolean getShooterOnTarget(double shooterAngle, double flywheelRpm){
        return Clamp.bounded(rc.shooterFlywheel.getRPM(), flywheelRpm-300, flywheelRpm+300)&& 
        Clamp.bounded(rc.shooter.getShooterAngle(), shooterAngle-3, shooterAngle+3) && 
        Clamp.bounded(rc.shooter.getState().velocity, -5, 5);
    }

}
