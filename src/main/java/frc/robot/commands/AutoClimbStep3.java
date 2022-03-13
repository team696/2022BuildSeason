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

public class AutoClimbStep3 extends CommandBase {
  Climber climber;
  Pneumatics pneumatics;
  DIOSub dioSub;
  boolean[] sensor;
  /** Creates a new AutoClimbStep2. */
  public AutoClimbStep3(Climber climber, Pneumatics pneumatics, DIOSub dioSub) {
    this.climber = climber;
    this.pneumatics = pneumatics;
    this.dioSub = dioSub;
    addRequirements(climber, pneumatics, dioSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.moveClimber(0.4); /* 0.4 */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pneumatics.autoPneumatics(LatchStates.DOUBLE_LATCHES, Value.kReverse);
    climber.moveClimber(0.2);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(!sensor[0] || !sensor[1] || !sensor[2] || !sensor[3]){
      return true;
    } 
    else{
      return false;
    }
  }
  
}
