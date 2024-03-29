// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class ShootCommand extends CommandBase {
  Shooter shooter;
  // double rpm;
  boolean isRunning;
  /** Runs the shooter motors based off of the distance from the limelight. */
  public ShootCommand(Shooter shooter, /*  double rpm,*/ boolean isRunning) {
    this.isRunning = isRunning;
    this.shooter = shooter;
    // this.rpm = rpm;
    addRequirements(shooter);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.isShooting = isRunning;
    double shootSpeed = shooter.shooterSpeed;
    System.out.println(shootSpeed);

    // shooter.setShooter(shooter.rpmToTalonFX(rpm));
    shooter.setShooter(shooter.rpmToTalonFX(shootSpeed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
