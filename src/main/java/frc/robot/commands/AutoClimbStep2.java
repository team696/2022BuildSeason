// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DIOSub;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Pneumatics.LatchStates;

public class AutoClimbStep2 extends CommandBase {
  Climber climber;
  Pneumatics pneumatics;
  DIOSub dioSub;
  double init_climber_pos;
  double current_climber_pos;
  static double target_climber_pos = 32 + 180;
  boolean[] sensor;
  /** Creates a new AutoClimbStep2. */
  public AutoClimbStep2(Climber climber, Pneumatics pneumatics, DIOSub dioSub) {
    this.climber = climber;
    this.pneumatics = pneumatics;
    this.dioSub = dioSub;
    addRequirements(climber, pneumatics, dioSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    init_climber_pos = climber.getClimberPos();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    current_climber_pos = -( (climber.getClimberPos() - init_climber_pos) / 130.66666666) -.25;
    SmartDashboard.putNumber("DinoCliomber", current_climber_pos);

    climber.moveClimber(-0.4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Math.abs(current_climber_pos - target_climber_pos) <= 20){
      return true;
    } 
    else{
      return false;
    }
  }
  
}
