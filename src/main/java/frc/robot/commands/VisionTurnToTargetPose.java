package frc.robot.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.FieldPosition;
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
    private DoubleSupplier xSpeed;
    private DoubleSupplier ySpeed;
    private DoubleSupplier rotSpeed;
    private Pose3d lobTarget = new Pose3d();
    double targetAngle = 0.0;

    public VisionTurnToTargetPose(
        DoubleSupplier xSpeed, 
        DoubleSupplier ySpeed, 
        DoubleSupplier rotSpeed,  
        ShooterVision shooterVision,
        Chassis chassis,
        AHRS gyro) {
        this.shooterVision = shooterVision;
        this.chassis = chassis;
        this.gyro = gyro;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rotSpeed = rotSpeed;
        
        addRequirements(shooterVision);
        addRequirements(chassis);
    }

    @Override
    public void initialize(){
        Alliance color = DriverStation.getAlliance().orElse(Alliance.Blue);
        lobTarget = color==Alliance.Red ? FieldPosition.RedLob : FieldPosition.BlueLob;
        //TODO: fiducial id filter
    }
    
    @Override
    public void execute()
    {
        Optional<ShooterVision.LimelightReadings> shooterData = shooterVision.getTargetDataOdometry(lobTarget);

        if (shooterData.isPresent()) {
            targetAngle = shooterData.get().angleHorizontal;
            targetAngle = Math.toRadians(targetAngle);
            //brian correctly pointed out that "rotspeed" is (-1..1) and thus is 1 degree bearing change, making it useless.
            chassis.driveToBearing(xSpeed.getAsDouble(), ySpeed.getAsDouble(), rotSpeed.getAsDouble() + targetAngle);
        }
        else{
        chassis.drive(xSpeed.getAsDouble(), ySpeed.getAsDouble(), rotSpeed.getAsDouble(), true,true);
        }
        
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
