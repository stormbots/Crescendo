// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ChassisConstants.DriveConstants;
import frc.robot.MAXSwerveModule;
import frc.robot.SwerveUtils;

public class Chassis extends SubsystemBase {
  /** Creates a new Chassis. 
//  * @param navx */
//   public Chassis(AHRS navx, SwerveDriveKinematics swerveDriveKinematics) {}

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }

  /** Creates a new Chassis. */
  // Create MAXSwerveModules
  private final MAXSwerveModule frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      0);

  private final MAXSwerveModule frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      0);

  private final MAXSwerveModule rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      0);

  private final MAXSwerveModule rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      0);

  // The gyro sensor
  public AHRS navx;
  public Field2d field = new Field2d();
  public SwerveDriveKinematics swerveDriveKinematics; 
  public SwerveDrivePoseEstimator swerveDrivePoseEstimator;

  // Slew rate filter variables for controlling lateral acceleration
  private double currentRotation = 0.0;
  private double currentTranslationDir = 0.0;
  private double currentTranslationMag = 0.0;

  private SlewRateLimiter magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double prevTime = WPIUtilJNI.now() * 1e-6;

  PIDController turnpid = new PIDController(1/Math.PI,0,0);
  //Good, 2/pi is too much, cause backlash

  public Chassis(AHRS navx, SwerveDriveKinematics swerveDriveKinematics, SwerveDrivePoseEstimator swerveDrivePoseEstimator) {
    this.navx = navx;
    this.swerveDriveKinematics = swerveDriveKinematics; 
    this.swerveDrivePoseEstimator = swerveDrivePoseEstimator;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  // Update the odometry in the periodic block
    swerveDrivePoseEstimator.update(
        navx.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        });

    field.setRobotPose(swerveDrivePoseEstimator.getEstimatedPosition());
    SmartDashboard.putNumber("/angle/rawnavx", navx.getAngle());
    SmartDashboard.putNumber("/angle/navxproccessed", navx.getRotation2d().getDegrees());
    SmartDashboard.putNumber("/angle/frmotor", frontRight.getState().angle.getDegrees());
    SmartDashboard.putNumber("/angle/flmotor", frontLeft.getState().angle.getDegrees());
    SmartDashboard.putNumber("/angle/brmotor", rearRight.getState().angle.getDegrees());
    SmartDashboard.putNumber("/angle/blmotor", rearLeft.getState().angle.getDegrees());

  }
  public Pose2d getPose() {
    return swerveDrivePoseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    swerveDrivePoseEstimator.resetPosition(
        navx.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        },
        pose);
  }
  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Percent output of the robot in the x direction (forward).
   * @param ySpeed        Percent output of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    SmartDashboard.putNumber("Raw X Speed", xSpeed);
    SmartDashboard.putNumber("Raw Y Speed", ySpeed);
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      SmartDashboard.putNumber("Input Translation Dir", inputTranslationDir);
      SmartDashboard.putNumber("Input Translation Mag", inputTranslationMag);

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / currentTranslationMag);
      } else {
        directionSlewRate = 500; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          currentTranslationMag = magLimiter.calculate(0.0);
        }
        else {
          currentTranslationDir = SwerveUtils.WrapAngle(currentTranslationDir + Math.PI);
          currentTranslationMag = magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(0.0);
      }
      prevTime = currentTime;
      
      xSpeedCommanded = currentTranslationMag * Math.cos(currentTranslationDir);
      ySpeedCommanded = currentTranslationMag * Math.sin(currentTranslationDir);
      currentRotation = rotLimiter.calculate(rot);

      SmartDashboard.putNumber("Current Translation Dir", currentTranslationDir);
      SmartDashboard.putNumber("Current Translation Mag", currentTranslationMag);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      currentRotation = rot;
    }

    SmartDashboard.putNumber("X Speed Out", xSpeedCommanded);
    SmartDashboard.putNumber("Y Speed Out", ySpeedCommanded);

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = currentRotation * DriveConstants.kMaxAngularSpeed;

    SmartDashboard.putNumber("X Speed Delivered", xSpeedDelivered);
    SmartDashboard.putNumber("Y Speed Delivered", ySpeedDelivered);

    var swerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, navx.getRotation2d())
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    rearLeft.setDesiredState(swerveModuleStates[2]);
    rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeft.resetEncoders();
    rearLeft.resetEncoders();
    frontRight.resetEncoders();
    rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    navx.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return navx.getRotation2d().getDegrees();
  }

  // PIDController turnpid = new PIDController(1/90.0,0,0);

  public void driveToBearing(double xSpeed, double ySpeed, double bearing){
    //move elsewhere
    turnpid.enableContinuousInput(0, Math.PI*2);
    // turnpid.atSetpoint();
    // turnpid.setTolerance(4);

    //In degrees
    double currentTheta = navx.getRotation2d().getRadians(); 
    // double thetaError = Math.toDegrees(MathUtil.angleModulus(rot - currentTheta.getRadians()));

    double output = turnpid.calculate(currentTheta,bearing);

    // if(thetaError > 180){  
    //   thetaError -= 360;
    // }
    // else if (thetaError < -180){
    //   thetaError += 360;
    // }
    // double proportionalRotation = thetaError / 180.0;
    // //may not be needed but just to be safe
    // if(proportionalRotation > 1.0){
    //   proportionalRotation = 1.0;
    // }

    drive(xSpeed, ySpeed, output, true, true);
    SmartDashboard.putNumber("bearing", bearing);
  }
}
