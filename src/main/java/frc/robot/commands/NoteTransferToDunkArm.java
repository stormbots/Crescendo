// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.stormbots.Clamp;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DunkArmRoller;
import frc.robot.subsystems.ShooterFlywheel;

public class NoteTransferToDunkArm extends Command {
  private ShooterFlywheel shooterFlywheel;
  private DunkArmRoller dunkArmRoller;
  final double distanceOfTransfer = 1.8;
  /** Creates a new NoteTransferToDunkArm. */
  public NoteTransferToDunkArm(ShooterFlywheel shooterFlywheel, DunkArmRoller dunkArmRoller) {
    // Use addRequirements() here to declare subsystem dependencies.\
    this.shooterFlywheel = shooterFlywheel;
    this.dunkArmRoller = dunkArmRoller;

    addRequirements(shooterFlywheel, dunkArmRoller);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // shooter flywheel diameter: 3 inch
    // dunkArm flywheel diameter: 1.375 inch
    dunkArmRoller.resetEncoder(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    dunkArmRoller.setPosition(distanceOfTransfer);
    shooterFlywheel.setRPM(500);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // TODO: Tune tol both
    var posTol = 0.5;
    var pos =
        Clamp.bounded(
            dunkArmRoller.getPosition(), distanceOfTransfer - posTol, distanceOfTransfer + posTol);

    SmartDashboard.putBoolean("dunkArmRoller/pos", pos);

    var velTol = 0.5; // per sec
    var vel = Clamp.bounded(dunkArmRoller.getVelocity(), -velTol, velTol);
    SmartDashboard.putBoolean("dunkArmRoller/vel", vel);
    return pos && vel;
  }
}
