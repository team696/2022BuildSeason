// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;
import frc.robot.Constants;

public class ClimbCommand2 extends CommandBase {
  
  Climber climber;
  // double percent;
  // boolean leftDir;
  // boolean rightDir;
  /** Open loop limber control. */
  public ClimbCommand2(Climber climber) {
    this.climber = climber;

    addRequirements(climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  climber.setNeutralBrake();
    double joystick_val = RobotContainer.controlPanel.getRawAxis(Constants.CLIMBER_MANUAL_ROTATION_AXIS);
    climber.moveClimber(joystick_val);

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