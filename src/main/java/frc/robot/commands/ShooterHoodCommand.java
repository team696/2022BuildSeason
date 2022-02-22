// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterHood;

public class ShooterHoodCommand extends CommandBase {
  ShooterHood shooterHood;
  double   position;
  double timer;
  /** Creates a new ShooterHoodCommand. */
  public ShooterHoodCommand(ShooterHood shooterHood, double  position) {
    this.shooterHood = shooterHood;
    this.position = position;
    addRequirements(shooterHood);
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
    timer++;
    shooterHood.moveActuators(position);

    // double hood_axis = RobotContainer.controlPanel.getRawAxis(1)*128*1.5;
    // last_hood_axis = hood_axis;
    // double delta = hood_axis - last_hood_axis;
    // if (delta + shooterHood.servoPosition() <= 100 && delta + shooterHood.servoPosition() >= 1){
    //   shooterHood.moveActuators(shooterHood.servoPosition()+delta);;
    // }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer > 150;
  }
}
