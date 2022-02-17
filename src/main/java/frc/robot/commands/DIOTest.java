// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DIOSub;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Pneumatics.LatchStates;

public class DIOTest extends CommandBase {
  Pneumatics pneumatics;
  DIOSub dioSub;
  double timer;
  boolean recent;
  /** Creates a new DIOTest. */
  public DIOTest(Pneumatics pneumatics, DIOSub dioSub) {
    this.pneumatics = pneumatics;
    this.dioSub = dioSub;
    addRequirements(pneumatics, dioSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    recent = false;
    timer = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    boolean[] test = dioSub.getSensorStates();
    if (!recent){
      if(test[0] == false || test[1] == false || test[2] == false || test[3] == false){
        
        pneumatics.autoPneumatics(LatchStates.DOUBLE_LATCHES, Value.kForward);
        recent = true;
      }
      else{
        pneumatics.autoPneumatics(LatchStates.DOUBLE_LATCHES, Value.kReverse);
      }
  }
  else{ 
    timer++;
    if (timer > Constants.CLIMBER_SENSOR_TIMEOUT_LOOPS){
      recent = false;
      timer = 0;
    }
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
