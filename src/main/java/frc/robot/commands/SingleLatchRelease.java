// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Pneumatics;

public class SingleLatchRelease extends CommandBase {

  Climber climber;
  Pneumatics pneumatics;
  double timer;
  /** Releases the single latches and applies a force to the bar to prevent buckling. */
  public SingleLatchRelease(Climber climber, Pneumatics pneumatics) {
    this.climber = climber;
    this.pneumatics = pneumatics;

    addRequirements(climber, pneumatics);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.lClimberMotor.setNeutralMode(NeutralMode.Brake);
    climber.rClimberMotor.setNeutralMode(NeutralMode.Brake);
    climber.moveClimber(-0.3 );
    pneumatics.movePneumatics2(Value.kReverse);
    timer++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {  
    climber.lClimberMotor.setNeutralMode(NeutralMode.Coast);;
    climber.rClimberMotor.setNeutralMode(NeutralMode.Coast);
    climber.moveClimber(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer>20;
  }
}
