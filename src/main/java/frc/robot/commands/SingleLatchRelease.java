// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class SingleLatchRelease extends CommandBase {

  Climber climber;
  double timer;
  /** Creates a new SingleLatchRelease. */
  public SingleLatchRelease(Climber climber) {
    this.climber = climber;
    addRequirements(climber);
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
    climber.moveClimber(0.1);
    climber.movePneumatics2(Value.kReverse);
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
    return timer>30;
  }
}
