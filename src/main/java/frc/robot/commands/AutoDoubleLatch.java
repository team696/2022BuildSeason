// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DIOSub;
import frc.robot.subsystems.LEDSub;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Pneumatics.LatchStates;

public class AutoDoubleLatch extends CommandBase {
  Pneumatics pneumatics;
  DIOSub dioSub;
  boolean[] sensor;
  LEDSub ledSub;

  /** Creates a new AutoDoubleLatch. */
  public AutoDoubleLatch(Pneumatics pneumatics, DIOSub dioSub, LEDSub ledSub) {
    this.dioSub = dioSub;
    this.pneumatics = pneumatics;
    this.ledSub = ledSub;
    addRequirements(pneumatics, dioSub, ledSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pneumatics.autoPneumatics(LatchStates.DOUBLE_LATCHES, Value.kForward);
    ledSub.setRightLEDs(255, 255, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sensor = dioSub.getSensorStates();


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pneumatics.autoPneumatics(LatchStates.DOUBLE_LATCHES, Value.kReverse);
    ledSub.setRightLEDs(0, 255, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!sensor[1] && !sensor[3]){
      return true;
    } 
    else{
      return false;
      
    }  }
}
