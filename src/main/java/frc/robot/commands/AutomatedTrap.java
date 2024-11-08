// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import com.stormbots.Clamp;
// import com.stormbots.LUT;

// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.units.Units;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.Climber;
// import frc.robot.subsystems.DunkArm;
// import frc.robot.subsystems.Chassis.ChassisOld;

// public class AutomatedTrap extends Command {
//   /** Creates a new AutomatedTrap. */

//   final double totalTime = 4;
//   ChassisOld chassis;
//   DunkArm dunkArm;
//   Climber climber;

//   SwerveDrivePoseEstimator pe;


//   Pose2d originalPose;
//   double initialTime =0;

//   Command pathCommand;

//   LUT climberLut = new LUT(new double[][]{
//     {0, 9},
//     {totalTime/5*1, 9},
//     {totalTime/10*9, 24}
//   });

//   LUT dunkArmLut = new LUT(new double[][]{
//     {0, 20},
//     {0.31, 90},
//     {0.4717, 110}
//   });

//   private double runtime(){
//     return Timer.getFPGATimestamp() - initialTime;
//   }
  

//   public AutomatedTrap(RobotContainer rc) {
  
//     this.chassis =rc.chassis;
//     this.dunkArm = rc.dunkArm;
//     this.climber =rc.climber;
//     this.pe = rc.swerveDrivePoseEstimator;

//     // Use addRequirements() here to declare subsystem dependencies.

//     addRequirements(chassis, dunkArm, climber);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     originalPose = pe.getEstimatedPosition();
//     initialTime = Timer.getFPGATimestamp();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if(runtime()<totalTime){
//       chassis.setModuleStates(new SwerveModuleState[]{
//         new SwerveModuleState(-0.51/totalTime, new Rotation2d()),
//         new SwerveModuleState(-0.51/totalTime, new Rotation2d()),
//         new SwerveModuleState(-0.51/totalTime, new Rotation2d()),
//         new SwerveModuleState(-0.51/totalTime, new Rotation2d())});
//     }
//     else{
//       chassis.drive(0, 0, 0, false, false);
//     }



//       var dist = pe.getEstimatedPosition().getTranslation().getDistance(originalPose.getTranslation());
//       dunkArm.setArmAngle(
//       Clamp.clamp(
//         dunkArmLut.get(dist)[0],
//        0, 110));

//        climber.setPosition(Units.Inches.of(climberLut.get(runtime())[0]));
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {


//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     // return false; 
//     return runtime() > totalTime;
//   }
// }
