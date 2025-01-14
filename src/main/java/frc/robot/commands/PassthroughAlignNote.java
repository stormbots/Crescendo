// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Clamp;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Passthrough;
import frc.robot.subsystems.PassthroughLock;

public class PassthroughAlignNote extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Passthrough passthrough;
  private final Intake intake;
  double stuckTimer = 0.0;
  double superStuckTimer = 0.5/2;
  double backwardTimer = 0.05*2;


  /** Creates a new PassthroughAlignNote. */
  public PassthroughAlignNote(Passthrough passthrough, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.passthrough = passthrough;
    this.intake = intake;

    addRequirements(passthrough);
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("unstick", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var dist = passthrough.getSensorDistance().in(Units.Inches);
    var nominal = 0.9+0.1;
    var kpassthrough = 0.1;
    var kintake = 0.1;

    var kpassresponse = (dist-nominal) * kpassthrough;
    var kintakeresponse = (dist-nominal) * kintake;
    SmartDashboard.putNumber("distMinusNominal", dist-nominal);
    kintakeresponse = Clamp.clamp(kintakeresponse, 0, 0.07);//0.05
    kpassresponse = Clamp.clamp(kpassresponse, -1, 0.05);

    if (dist<nominal) {
      kintakeresponse=0;
      PassthroughLock.getInstance().unlock();
    }

    if (passthrough.isBlocked()) {
      kpassresponse = Clamp.clamp(kpassresponse, -1, 0.05);
      kintakeresponse = Clamp.clamp(kintakeresponse, 0, 0.04);
    }


    if ( !passthrough.isBlocked() && intake.getVelocity() < 1000) {
      kintake=1; //known power that moves it if blocked
      SmartDashboard.putBoolean("unstick", true);
      //stuck timer start if at 
      // if(timer!=0) timer = Timer.getFPGATimestamp()
      if(stuckTimer != 0){
        stuckTimer = Timer.getFPGATimestamp();
      }
    }
    else {
      // stuck timer clear
      stuckTimer = 0.0;
      SmartDashboard.putBoolean("unstick", false);
    }

    //act on stuck timer:
    //after X time, and before X+Y time, reverse
    if(Clamp.bounded(stuckTimer, stuckTimer+superStuckTimer, stuckTimer+superStuckTimer+backwardTimer)) {
      kintakeresponse = -1.0;
    }
    SmartDashboard.putNumber("timer/superStuckTimer", superStuckTimer);
    SmartDashboard.putNumber("timer/stuckTimer", stuckTimer);
    SmartDashboard.putNumber("timer/backwardTimer", backwardTimer);

    SmartDashboard.putNumber("intakeResponse",kintakeresponse);
    SmartDashboard.putNumber("passthroughresponse",kpassresponse);



    intake.setPower(kintakeresponse);
    passthrough.setPower(kpassresponse);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    PassthroughLock.getInstance().unlock();
    intake.stop();
    passthrough.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
    // return passthrough.getSensorDistance().in(Units.Inches) > 6;
  }
}
