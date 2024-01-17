// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

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

}
