// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DunkArm;

/** An example command that uses an example subsystem. */
public class SetDunkArmProfiled extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DunkArm dunkArm;
  private double armAngle;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(45, 45);
  TrapezoidProfile.State goal = new TrapezoidProfile.State(0, 0);
  TrapezoidProfile.State initial = new TrapezoidProfile.State(0, 0);
  TrapezoidProfile armProfile = new TrapezoidProfile(constraints);
  double startTimer = 0;
  

  public SetDunkArmProfiled(double armAngle, DunkArm dunkArm) {
    this.armAngle = armAngle;
    this.dunkArm = dunkArm;
    addRequirements(dunkArm);

    goal = new TrapezoidProfile.State(armAngle, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTimer = Timer.getFPGATimestamp();
    initial = dunkArm.getState();
    armProfile = new TrapezoidProfile(constraints);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var currentState = dunkArm.getState();

    var targetState = armProfile.calculate(Timer.getFPGATimestamp()-startTimer, currentState, goal);
    var targetPosition = targetState.position;
    dunkArm.setArmAngle(targetPosition);
    SmartDashboard.putNumber("dunkprofile/target", targetPosition);
    SmartDashboard.putNumber("dunkprofile/targetVel", targetState.velocity);
    
    SmartDashboard.putNumber("dunkprofile/current", currentState.position);
    SmartDashboard.putNumber("dunkprofile/velocity", currentState.velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Will need to figure out, add feedforward stuff?
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
