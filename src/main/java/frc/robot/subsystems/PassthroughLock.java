package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PassthroughLock extends SubsystemBase{
    public Servo servo = new Servo(3);
    private static PassthroughLock instance = new PassthroughLock();

    private PassthroughLock() {
        servo.setBoundsMicroseconds(2500, 1501, 1500, 1499, 500);  
        
        // this.setDefaultCommand(new RunCommand(this::unlock,this));
    }

    public static PassthroughLock getInstance(){
        if(instance==null) instance = new PassthroughLock();
        return instance;
      }

    public void lock() {
        servo.setPulseTimeMicroseconds(1500);
    }

    public void unlock() {
        servo.setPulseTimeMicroseconds(2000);
    }

    public static Command setUnlocked(){
        getInstance(); //make sure instance exists
        return new InstantCommand(instance::unlock,instance);
    }

    public static Command setLocked() {
        getInstance();
        return new InstantCommand(instance::lock, instance);
    }
}