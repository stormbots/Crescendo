// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/** Add your docs here. */
public class AutoFactory {
    RobotContainer rc;

    public AutoFactory(RobotContainer rc){
        this.rc = rc;
    }

    public Command ExampleAuto(){
        return new InstantCommand(()->{},rc.chassis,rc.climber);
    }

}
