// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Chassis;

import java.sql.Driver;
import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase.IdleMode;

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
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.SwerveUtils;
import frc.robot.subsystems.Chassis.ChassisConstants.DriveConstants;
import frc.robot.subsystems.Chassis.ChassisConstants.OIConstants;

public class Chassis extends SubsystemBase {
  /** Creates a new Chassis. 
//  * @param navx */
//   public Chassis(AHRS navx, SwerveDriveKinematics swerveDriveKinematics) {}

  /** Creates a new Chassis. */
  // Create MAXSwerveModules
  private final MAXSwerveModule frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      0,
      new SimpleMotorFeedforward(0.080663, 1.9711, 0.36785));

  private final MAXSwerveModule frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      0,
      new SimpleMotorFeedforward(0.12682, 1.971, 0.30547));

  private final MAXSwerveModule rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      0,
      new SimpleMotorFeedforward(0.11553, 1.956, 0.33358));

  private final MAXSwerveModule rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      0,
      new SimpleMotorFeedforward(0.11616, 1.9571, 0.321));

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

  /**
   * Tuning notes: 
   * 1/pi is good
   * 2/pi is too much, cause backlash 
   */
  PIDController turnpid = new PIDController(1/Math.PI,0,0.01);

  private final SysIdRoutine sysIdRoutine =
  new SysIdRoutine(
      // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          // Tell SysId how to plumb the driving voltage to the motors.
          (Measure<Voltage> volts) -> {
            frontLeft.setVoltageDrive(volts);
            frontRight.setVoltageDrive(volts);
            rearLeft.setVoltageDrive(volts);
            rearRight.setVoltageDrive(volts);
          },
          log -> {
            log.motor("frontRight")
                .voltage(Units.Volts.of(frontRight.getPowerOutput()))
                .linearPosition(Units.Meters.of(frontRight.getDrivingEncoderPosition()))
                .linearVelocity( Units.MetersPerSecond.of(frontRight.getState().speedMetersPerSecond));
            log.motor("frontLeft")
              .voltage(Units.Volts.of(frontLeft.getPowerOutput()))
              .linearPosition(Units.Meters.of(frontLeft.getDrivingEncoderPosition()))
              .linearVelocity( Units.MetersPerSecond.of(frontLeft.getState().speedMetersPerSecond));
            log.motor("rearRight")
              .voltage(Units.Volts.of(rearRight.getPowerOutput()))
              .linearPosition(Units.Meters.of(rearRight.getDrivingEncoderPosition()))
              .linearVelocity( Units.MetersPerSecond.of(rearRight.getState().speedMetersPerSecond));
            log.motor("rearLeft")
              .voltage(Units.Volts.of(rearLeft.getPowerOutput()))
              .linearPosition(Units.Meters.of(rearLeft.getDrivingEncoderPosition()))
              .linearVelocity( Units.MetersPerSecond.of(rearLeft.getState().speedMetersPerSecond));
          },
          this));

  public Chassis(AHRS navx, SwerveDriveKinematics swerveDriveKinematics, SwerveDrivePoseEstimator swerveDrivePoseEstimator, Field2d field) {
    this.navx = navx;
    this.swerveDriveKinematics = swerveDriveKinematics; 
    this.swerveDrivePoseEstimator = swerveDrivePoseEstimator;

    turnpid.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  // Update the odometry in the periodic block
    
    //i WILL cry if this doesn't work by next week
    var pose = swerveDrivePoseEstimator.getEstimatedPosition();
    SmartDashboard.putNumber("chassis/x",pose.getX());
    SmartDashboard.putNumber("chassis/y",pose.getY());
    field.getObject("gyro").setPose(new Pose2d(5,5, navx.getRotation2d()));
    field.setRobotPose(pose);
    
    SmartDashboard.putData("chassis", field);
    SmartDashboard.putData("modules/fr", frontRight);
    SmartDashboard.putData("modules/fl", frontLeft);
    SmartDashboard.putData("modules/rr", rearRight);
    SmartDashboard.putData("modules/rl", rearLeft);

    SmartDashboard.putNumber("modules/frTurn", frontRight.getPosition().angle.getDegrees());
    SmartDashboard.putNumber("modules/flTurn", frontLeft.getPosition().angle.getDegrees());
    SmartDashboard.putNumber("modules/rrTurn", rearRight.getPosition().angle.getDegrees());
    SmartDashboard.putNumber("modules/rlTurn", rearLeft.getPosition().angle.getDegrees());

    SmartDashboard.putNumber("modules/frVel", frontRight.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("modules/flVel", frontLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("modules/BrVel", rearRight.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("modules/BlVel", rearLeft.getState().speedMetersPerSecond);
  
    SmartDashboard.putNumber("modules/frVelDiff", frontRight.getDesiredSate().speedMetersPerSecond - frontRight.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("modules/flVelDiff", frontLeft.getState().speedMetersPerSecond - frontLeft.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("modules/BrVelDiff", rearRight.getState().speedMetersPerSecond - rearRight.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("modules/BlVelDiff", rearLeft.getState().speedMetersPerSecond - rearLeft.getState().speedMetersPerSecond);
    
    SmartDashboard.putNumber("modules/frCurr", frontRight.getDriveCurrent());
    SmartDashboard.putNumber("modules/flCurr", frontLeft.getDriveCurrent());
    SmartDashboard.putNumber("modules/BrCurr", rearRight.getDriveCurrent());
    SmartDashboard.putNumber("modules/BlCurr", rearLeft.getDriveCurrent());

    SmartDashboard.putNumber("modules/frPower", frontRight.getPowerOutput());
    SmartDashboard.putNumber("modules/flPower", frontLeft.getPowerOutput());
    SmartDashboard.putNumber("modules/BrPower", rearRight.getPowerOutput());
    SmartDashboard.putNumber("modules/BlPower", rearLeft.getPowerOutput());

    // SmartDashboard.putNumber("/angle/rawnavx", navx.getAngle());
    SmartDashboard.putNumber("/angle/navxproccessed", navx.getRotation2d().getDegrees());

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

    // var rot = new Rotation2d(MathUtil.angleModulus(navx.getRotation2d().getRadians()));
    var rot = navx.getRotation2d(); 

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

  public void resetOdometryAllianceManaged(Pose2d pose){
    if(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red){
      pose = new Pose2d(16.542-pose.getX(), pose.getY(), new Rotation2d(Math.PI-pose.getRotation().getRadians())); 
    }

    var rot = navx.getRotation2d(); 

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

    // SmartDashboard.putNumber("Raw X Speed", xSpeed);
    // SmartDashboard.putNumber("Raw Y Speed", ySpeed);
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // SmartDashboard.putNumber("Input Translation Dir", inputTranslationDir);
      // SmartDashboard.putNumber("Input Translation Mag", inputTranslationMag);

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

      // SmartDashboard.putNumber("Current Translation Dir", currentTranslationDir);
      // SmartDashboard.putNumber("Current Translation Mag", currentTranslationMag);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      currentRotation = rot;
    }

    // SmartDashboard.putNumber("X Speed Out", xSpeedCommanded);
    // SmartDashboard.putNumber("Y Speed Out", ySpeedCommanded);

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = currentRotation * DriveConstants.kMaxAngularSpeed;

    // SmartDashboard.putNumber("X Speed Delivered", xSpeedDelivered);
    // SmartDashboard.putNumber("Y Speed Delivered", ySpeedDelivered);

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

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }
  
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
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
   * Sets the swerve ModuleStates. Meters Per Second
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

  public SwerveModuleState[] getModuleStates(){
    return new SwerveModuleState[] {
      frontLeft.getState(),
      frontRight.getState(),
      rearLeft.getState(),
      rearRight.getState()
    };
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    rearLeft.resetEncoders();
    rearRight.resetEncoders();
  }
  
  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    navx.reset();
  }


  /** Apply an offset from initial navx zero to the intended "forward" direction
   * for fieldcentric controls.
   * positive value rotates zero CW
   * 
   * @param isBlue Control over whether bot is blue, makes overload backwards compatible
   */
  public void setFieldCentricOffset(double degrees, BooleanSupplier isBlue){
    zeroHeading();
    navx.setAngleAdjustment(isBlue.getAsBoolean() ? degrees : -degrees);
  }

  /** Apply an offset from initial navx zero to the intended "forward" direction
   * for fieldcentric controls.
   * positive value rotates zero CW
   * In degrees
   */
  public void setFieldCentricOffset(double offset){
    setFieldCentricOffset(offset, ()->true);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return navx.getRotation2d().getDegrees();
  }

  public Command getZeroOutputCommand(){
    return new RunCommand(()->drive(0,0,0, false, false), this);
  }

  public void driveToBearing(double xSpeed, double ySpeed, double bearingRad){
    //move elsewhere
    // turnpid.atSetpoint();
    // turnpid.setTolerance(4);

    //In degrees
    double currentTheta = navx.getRotation2d().getRadians(); 
    bearingRad = MathUtil.angleModulus(bearingRad);
    // double thetaError = Math.toDegrees(MathUtil.angleModulus(rot - currentTheta.getRadians()));

    double output = turnpid.calculate(currentTheta,bearingRad);

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
    // SmartDashboard.putNumber("bearing", bearingRad);
  }
  
  public void driveToBearing(double radians){
    driveToBearing(0, 0, radians);
  }
  

  //TODO: not working
  /**
   * Fieldcentric drive command, using Field coordinates 
   * @param xPower [-1..1] with positive away from driver
   * @param yPower [-1..1] with positive to left
   * @param rotationPower [-1..1] with positive CCW
   * @return
   */
  public Command getFCDriveCommand(DoubleSupplier xPower, DoubleSupplier yPower, DoubleSupplier rotationPower){
    return new RunCommand(
      () -> {drive(
          MathUtil.applyDeadband(xPower.getAsDouble(), OIConstants.kDriveDeadband),
          MathUtil.applyDeadband(yPower.getAsDouble(), OIConstants.kDriveDeadband),
          MathUtil.applyDeadband(rotationPower.getAsDouble(), OIConstants.kDriveDeadband),
          true, true);},
      this);
  }

  public Command getDriveToBearingCommand(DoubleSupplier xPower, DoubleSupplier yPower, Supplier<Measure<Angle>> bearing){
    return new RunCommand(
      () -> {
        driveToBearing(
          MathUtil.applyDeadband(xPower.getAsDouble(), OIConstants.kDriveDeadband),
          MathUtil.applyDeadband(yPower.getAsDouble(), OIConstants.kDriveDeadband),
          bearing.get().in(Units.Radians)
        );
      },
      this);
  }

  public void setIdleMode(IdleMode idleMode){
    for(MAXSwerveModule module : new MAXSwerveModule[]{frontLeft, frontRight, rearLeft, rearRight}){
      module.drivingSparkFlex.setIdleMode(idleMode);
    }
  }

  public Measure<Distance> getDistanceFromStageCenter(){
    Pose2d currentPose = swerveDrivePoseEstimator.getEstimatedPosition();

    boolean isBlue = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;

    double stageCenterX = 4.9;
    if(!isBlue){stageCenterX = 16.542-stageCenterX;}
    double stageCenterY = 4.1; 
    
    double distanceInMeters = Math.hypot(currentPose.getX()-stageCenterX, currentPose.getY()-stageCenterY);

    return Units.Meters.of(distanceInMeters);
  }
  
  public void setCurrentLimits(double amps){
    amps /= 4;
    frontLeft.setCurrentLimit(amps); 
    frontRight.setCurrentLimit(amps);
    rearLeft.setCurrentLimit(amps);
    rearRight.setCurrentLimit(amps);
  }

  public void updateOdometry(){
    swerveDrivePoseEstimator.updateWithTime(
        Timer.getFPGATimestamp(),
        navx.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        });

    // System.out.println(Timer.getFPGATimestamp());
  }
}
