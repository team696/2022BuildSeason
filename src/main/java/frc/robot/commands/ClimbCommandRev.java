// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSub;

public class ClimbCommandRev extends CommandBase {
  ClimberSub climberSub;
  double percent;

  /** Creates a new ClimbCommandRev. */
  public ClimbCommandRev(ClimberSub climberSub, double percent) {
    this.climberSub = climberSub;
    this.percent = percent;
    addRequirements(climberSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climberSub.lClimberMotor.setInverted(false);
    climberSub.rClimberMotor.setInverted(true);
    climberSub.moveClimber(percent);
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
