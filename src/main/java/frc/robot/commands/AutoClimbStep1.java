// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DIOSub;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Pneumatics.LatchStates;

public class AutoClimbStep1 extends CommandBase {
  Climber climber;
  Pneumatics pneumatics; 
  DIOSub dioSub;
  boolean[] sensor;

  /** Creates a new AutoClimbStep1. */
  public AutoClimbStep1(Climber climber, Pneumatics pneumatics, DIOSub dioSub) {
    this.climber = climber;
    this.pneumatics = pneumatics;
    this.dioSub = dioSub;
    addRequirements(climber, pneumatics, dioSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pneumatics.autoPneumatics(LatchStates.DOUBLE_LATCHES, Value.kForward);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    sensor = dioSub.getSensorStates();

    // while(climber.getClimberVoltage() < Constants.CLIMBER_MAX_VOLTAGE){
      climber.moveClimber(-0.55);
    // }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    // pneumatics.autoPneumatics(LatchStates.DOUBLE_LATCHES, Value.kReverse);
    climber.moveClimber(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
        
    if(!sensor[4] || !sensor[5]){
      return true;
    } 
    else{
      return false;
      
    }
  }
}
