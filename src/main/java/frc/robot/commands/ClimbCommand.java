// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ClimbCommand extends CommandBase {
  
  Climber climber;
  // double percent;
  // boolean leftDir;
  // boolean rightDir;
  /** Creates a new ClimbCommand1. */
  public ClimbCommand(Climber climber) {
    // this.percent = percent;
    this.climber = climber;
    // this.leftDir = leftDir;
    // this.rightDir = rightDir;

    addRequirements(climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

climber.moveClimber(RobotContainer.controlPanel.getRawAxis(0));
System.out.println(RobotContainer.controlPanel.getRawAxis(0));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // climber.moveClimber(0);\
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
