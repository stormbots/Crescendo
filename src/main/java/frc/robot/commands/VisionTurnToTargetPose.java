package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.studica.frc.AHRS;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.FieldPosition.TargetType;
import frc.robot.subsystems.IntakeVision;
import frc.robot.subsystems.ShooterVision;

public class VisionTurnToTargetPose extends Command{
    private IntakeVision intakeVision;
    private ShooterVision shooterVision;
    private Chassis chassis;
    private AHRS gyro;
    private TargetType targetType;
    private Field2d field = new Field2d();
    private SwerveDrivePoseEstimator swervePE;
    private DoubleSupplier xSpeed;
    private DoubleSupplier ySpeed;
    private DoubleSupplier rotSpeed;
    private Pose2d lobTarget = new Pose2d();
    double targetAngle = 0.0;
    Rotation2d offset = new Rotation2d();

    public VisionTurnToTargetPose(
        DoubleSupplier xSpeed, 
        DoubleSupplier ySpeed, 
        DoubleSupplier rotSpeed,  
        ShooterVision shooterVision,
        Chassis chassis,
        AHRS gyro,
        SwerveDrivePoseEstimator swervePE, Field2d field) {
        this.shooterVision = shooterVision;
        this.chassis = chassis;
        this.gyro = gyro;
        this.swervePE = swervePE;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rotSpeed = rotSpeed;
        this.field = field;
        
        addRequirements(shooterVision);
        addRequirements(chassis);
    }

    @Override
    public void initialize(){
        Alliance color = DriverStation.getAlliance().orElse(Alliance.Blue);
        // lobTarget = color==Alliance.Red ? FieldPosition.RedLob : FieldPosition.BlueLob;
        lobTarget = color==Alliance.Red ? field.getObject("Red Lob").getPose() : field.getObject("Blue Lob").getPose();
        //TODO: fiducial id filter
        field.getObject("Robot").setPose(swervePE.getEstimatedPosition());
        
        SmartDashboard.putData("visionturnpose",field);
        
    }
    
    @Override
    public void execute()
    {
        Pose2d orthognalAngle = shooterVision.getPoseDifference(lobTarget).get(); //NOT the orthognal angle
           
        //brian correctly pointed out that "rotspeed" is (-1..1) and thus is 1 degree bearing change, making it useless.
        // chassis.driveToBearing(xSpeed.getAsDouble(), ySpeed.getAsDouble(), rotSpeed.getAsDouble() + targetAngle);
        // targetAngle = orthognalAngle.getTranslation().getAngle().getRadians()*2;
        // chassis.driveToBearing(xSpeed.getAsDouble(), ySpeed.getAsDouble(), targetAngle);


        var targetangle = gyro.getRotation2d().getDegrees();
        targetangle = targetangle  + orthognalAngle.getTranslation().getAngle().getDegrees();
        targetangle = Math.toRadians(targetangle+offset.getDegrees());
        chassis.driveToBearing(xSpeed.getAsDouble(), ySpeed.getAsDouble(), targetangle);
        
        SmartDashboard.putNumber("shootervision/targetAngle", targetAngle);  
        // Optional<Double> orthogonalAngleFR = shooterVision.getOrthogonalAngleFR(lobTarget);
        SmartDashboard.putNumber("shootervision/targetanglefr", orthognalAngle.getTranslation().getAngle().getDegrees());
        Optional<Double> distance = shooterVision.getDistance(lobTarget);
        SmartDashboard.putNumber("shootervision/distancefromtargetpose", distance.get());
    
        field.getObject("delta").setPose(orthognalAngle);
        field.getObject("Robot").setPose(swervePE.getEstimatedPosition());
        SmartDashboard.putData("visionturnpose",field);
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished()
    {
        return false;
    }

    public VisionTurnToTargetPose reverseDirection(){
        offset = new Rotation2d(Math.PI);
        return this;
    }
}
