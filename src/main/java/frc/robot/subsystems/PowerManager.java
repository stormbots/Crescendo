// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Optional;
import java.util.function.DoubleConsumer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PowerManager extends SubsystemBase implements Sendable {
  private static PowerManager instance;
  
  /** The total system current budget, which will be assigned to everything */
  private double robotPowerBudget=280;

  /** The remaining current that hasn't been assigned yet */
  private double remainingPower=robotPowerBudget;

  /** Container class to keep all the info on a system in one place */
  private class SystemInfo{
    public double ampsMaximum;
    public double ampsCurrentlyAllocated;
    public Optional<Command> runningCommand;
    public SubsystemBase system;
  }

  /** A specialized version for the system that may have it's current draw adjusted */
  private class PowerSinkInfo extends SystemInfo{
    public DoubleConsumer setPower;
    public double ampsMinimum;
  }

  private HashMap<SubsystemBase,SystemInfo> systems = new HashMap<>(10);
  private Optional<PowerSinkInfo> powerSink;


  /** Creates a new PowerMonitor. */
  private PowerManager() {
    this.powerSink = Optional.empty();
  }

  public static PowerManager getInstance(){
    if(instance==null) instance = new PowerManager();

    return instance;
  }

  public PowerManager setRobotPowerBudget(double amps){
    this.robotPowerBudget = amps;
    this.remainingPower = this.robotPowerBudget;
    return this;
  }

  public PowerManager addSystem(SubsystemBase system, double ampsWhenRunning){
    var info = new SystemInfo();
    info.ampsCurrentlyAllocated=0;
    info.ampsMaximum=ampsWhenRunning;
    info.runningCommand=Optional.empty();
    info.system = system;
    this.systems.put(system, info);
    return this;
  }

  public PowerManager addChassisSystem(SubsystemBase system, double ampsMinimum, double ampsMaximum, DoubleConsumer setCurrentLimit){
    var info = new PowerSinkInfo();
    info.ampsCurrentlyAllocated=0;
    info.ampsMinimum=ampsMinimum;
    info.ampsMaximum=ampsMaximum;
    info.runningCommand=Optional.empty();
    info.setPower=setCurrentLimit;
    info.system = system;

    this.powerSink = Optional.of(info);
    return this;
  }

  /**manually specify the power used by the subsystem's current command
   * This should probably be added to the subsystem's default stop(); command,
   * or called after a defaultCommand puts it in a low power position/state
  */
  public void setPowerDraw( double ampsToAllocate,SubsystemBase system){
    var info = systems.get(system);
    // do nothing if the system is not registered yet;
    // This could happen if the subsystem was never registered, or if the 
    // PowerMonitor is declared after the subsystem of interest
    if(info==null) return;

    this.remainingPower += info.ampsCurrentlyAllocated - ampsToAllocate;
    info.ampsCurrentlyAllocated = ampsToAllocate;
    info.runningCommand = Optional.ofNullable(system.getCurrentCommand());
  }

  /** Specify the power draw for a command. Useful for sequences or parallel
   * groups where a power adjustment is desirable
   */
  public Command setPowerDrawCommand(double ampsToAllocate,SubsystemBase system){
    return new InstantCommand(()->setPowerDraw(ampsToAllocate,system));
  }


  @Override
  public void periodic() {
    //No reason to do any work if we don't have a system to control
    if(powerSink.isEmpty())return;
 
    // scan through subsystems, look for changes in currently running command
    // figure out system power draw
    // figure out remaining power budget
    // cap chassis with the appropriate power constraints if the budget has changed
    systems.forEach(this::checkRunningCommands);

    var sink = this.powerSink.get();
    var sinkpower = MathUtil.clamp(remainingPower,sink.ampsMinimum,sink.ampsMaximum);

    if(sink.ampsCurrentlyAllocated != sinkpower){
      sink.ampsCurrentlyAllocated = sinkpower;
      sink.setPower.accept(sinkpower);
    }
  }

  private void checkRunningCommands(SubsystemBase system, SystemInfo info){
    var command = Optional.ofNullable(system.getCurrentCommand());

    //NOTE: This may be weird, but simplifies optional unwrapping.
    // We don't care about the resulting values, only if they're dissimilar
    if(info.runningCommand.orElse(null) != command.orElse(null)){
      // System.out.printf("Change! %s -> %d ",system.getName(), (int)info.ampsCurrentlyAllocated);
      //something changed! De-allocate any power provided to it
      this.remainingPower += info.ampsCurrentlyAllocated; 
      info.ampsCurrentlyAllocated=0;

      //if appropriate, re-allocate the new power.
      if(command.isPresent()){
        // System.out.printf(" -> %d \n",(int)info.ampsMaximum);
        this.remainingPower -= info.ampsMaximum;
        info.ampsCurrentlyAllocated= info.ampsMaximum;
      }
      // else{
        // System.out.printf("-> 0\n");
      // }

      info.runningCommand = command;
    }
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("PowerMonitor");
    
    builder.addDoubleProperty("Power Managed",()->this.robotPowerBudget,(f)->this.robotPowerBudget=f);
    builder.addDoubleProperty("Power to systems",()->this.robotPowerBudget-this.remainingPower,null);
    builder.addDoubleProperty("Power to sink",()->this.remainingPower,null);

    if(powerSink.isPresent()){
      var sink = powerSink.get();
      builder.addDoubleProperty(
        sink.system.getName(),
        ()->sink.ampsCurrentlyAllocated,
        (f)->sink.ampsCurrentlyAllocated=f
      );
    }

    systems.forEach((sys,info)->{
      builder.addDoubleProperty(
        sys.getName(), 
        ()->systems.get(sys).ampsCurrentlyAllocated,
        (f)->systems.get(sys).ampsCurrentlyAllocated=f
      );
    });
  }

}