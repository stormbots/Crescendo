// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chassis;

public class FollowPath extends Command {
  private final Timer timer = new Timer();

  private Chassis chassis;

  private PathPlannerPath path;
  private PathPlannerTrajectory trajectory;

  private final PIDController rotationPID = new PIDController(2/Math.PI, 0, 0);
  private final PIDController translationPID = new PIDController(1, 0, 0);

  //Unfortunately, i have no other way other than to track the states by index
  int index = 0;
  List<PathPlannerTrajectory.State> states;

  
  /** Creates a new FollowPath. */
  public FollowPath(PathPlannerPath path, Chassis chassis) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.chassis = chassis;
    this.path = path;

    addRequirements(chassis);
  }

  public FollowPath(String choreoTraj, Chassis chassis){
    this(PathPlannerPath.fromChoreoTrajectory(choreoTraj), chassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotationPID.enableContinuousInput(-Math.PI, Math.PI);
    trajectory = new PathPlannerTrajectory(path, chassis.getChassisSpeeds(), chassis.getRotation2d());
    states = trajectory.getStates();
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentTime = timer.get();

    PathPlannerTrajectory.State desiredState = trajectory.sample(currentTime);
    PathPlannerTrajectory.State nextDesiredState;



    if(index+1 < states.size() && states.get(index).equals(desiredState)){//current indexed state is desired state
      nextDesiredState = states.get(index+1);
    }

    else if (index+2 < states.size() && states.get(index+1).equals(desiredState)){ //state of next index is desired state
      nextDesiredState = states.get(index+2);
      index++;
    }

    else{
      nextDesiredState = trajectory.getEndState();
    }


    Rotation2d desiredHeading = desiredState.heading;

    Pose2d currentPose = chassis.getPose();

    double vx = desiredState.velocityMps * Math.cos(desiredHeading.getRadians());
    double vy = desiredState.velocityMps * Math.sin(desiredHeading.getRadians());
    double vTheta;
    double aTheta;
    if(desiredState.holonomicAngularVelocityRps.isPresent()){
      vTheta=desiredState.holonomicAngularVelocityRps.get();
      aTheta = (nextDesiredState.holonomicAngularVelocityRps.get() - desiredState.holonomicAngularVelocityRps.get())/desiredState.timeSeconds; //Doesn't have desiredState.angularAcceleration
    }
    else{
      vTheta=0;
      aTheta=0;
    }

    ChassisSpeeds realSpeeds = chassis.getChassisSpeeds();
    SmartDashboard.putNumber("/autos/atheta", aTheta);
    SmartDashboard.putNumber("/autos/xyVelError", Math.abs(desiredState.velocityMps-Math.hypot(realSpeeds.vxMetersPerSecond, realSpeeds.vyMetersPerSecond)));
    SmartDashboard.putNumber("/autos/xyerror", Math.abs(Math.hypot(desiredState.positionMeters.getX()-chassis.getPose().getX(), desiredState.positionMeters.getY()-chassis.getPose().getY())));

    SmartDashboard.putNumber("/autos/thetaVelError", Math.abs(desiredState.holonomicAngularVelocityRps.get()-realSpeeds.omegaRadiansPerSecond));
    SmartDashboard.putNumber("/autos/thetaerror", Math.abs(desiredState.heading.getRadians()-chassis.getPose().getRotation().getRadians()));


    double ax = desiredState.accelerationMpsSq * Math.cos(desiredHeading.getRadians());
    double ay = desiredState.accelerationMpsSq * Math.sin(desiredHeading.getRadians());
  
    double xFeedback = translationPID.calculate(currentPose.getX(), desiredState.getTargetHolonomicPose().getX());
    double yFeedback = translationPID.calculate(currentPose.getY(), desiredState.getTargetHolonomicPose().getY());
    double thetaFeedback = rotationPID.calculate(currentPose.getRotation().getRadians(), desiredState.targetHolonomicRotation.getRadians());

    //TEMP:
    // xFeedback = 0;
    // yFeedback = 0;
    // thetaFeedback = 0;

    ChassisSpeeds chassisVelocity = ChassisSpeeds.fromFieldRelativeSpeeds(vx + xFeedback, vy + yFeedback, vTheta + thetaFeedback, currentPose.getRotation());

    ChassisSpeeds chassisAcceleration = ChassisSpeeds.fromFieldRelativeSpeeds(ax, ay, aTheta, currentPose.getRotation());
    
    chassis.setModuleStates(chassisVelocity, chassisAcceleration);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.timer.stop(); // Stop timer
    chassis.setModuleStates(new ChassisSpeeds(), new ChassisSpeeds()); // Stop motors
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //We have room to work with this one. If we always lag behind, itd be nice for the pid to correct so some extra time would be nice.
    return timer.hasElapsed(trajectory.getTotalTimeSeconds());
  }
}
