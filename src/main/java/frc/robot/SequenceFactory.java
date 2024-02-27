// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.NoteTransferToDunkArm;
import frc.robot.commands.SetDunkArmSlew;
import frc.robot.commands.SetShooterProfiled;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.PassthroughAlignNote;

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
            new SetShooterProfiled(0, rc.shooter)
        )
        .andThen(
            new ParallelCommandGroup(
                new RunCommand(()->rc.dunkArmRoller.setSpeed(0.1), rc.dunkArmRoller),
                new RunCommand(()->rc.passthrough.intake(), rc.passthrough),
                new RunCommand(()->rc.intake.intake(), rc.intake),
                rc.shooterFlywheel.getShooterSetRPMCommand(3000)
            ).until(()->rc.passthrough.isBlocked()==false)
        )
        .andThen(
            new NoteTransferToDunkArm(rc.shooterFlywheel, rc.dunkArmRoller),
            new InstantCommand(()->rc.intake.stop(), rc.intake),
            new InstantCommand(()->rc.passthrough.stop(), rc.passthrough)
        );
    }

    public Command getSetRPMandShootCommand(double rpm, double angle){
        return new InstantCommand()
        .andThen(new ParallelCommandGroup(
            rc.shooterFlywheel.getShooterSetRPMCommand(rpm),
            new SetShooterProfiled(angle, rc.shooter).runForever()
            )
            // .withTimeout(5)
            .until(
                ()->{return rc.shooterFlywheel.isOnTarget() && rc.shooter.isOnTarget();}
            )
        )
        .andThen(
            new RunCommand(rc.passthrough::intake,rc.passthrough)
            .alongWith(new RunCommand(rc.intake::intake,rc.intake))
            .withTimeout(1)
        );
        // .andThen(new SetShooterProfiled(0, rc.shooter));
    }

    public Command getIntakeThenAlignCommand(){
        return new InstantCommand()
            .andThen(new IntakeNote(rc.intake, rc.passthrough))
            .andThen(new PassthroughAlignNote(rc.passthrough, rc.intake));
    }

}
