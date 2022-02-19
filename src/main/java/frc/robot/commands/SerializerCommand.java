// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Serializer;

public class SerializerCommand extends CommandBase {
  Serializer serializer;
  double percent1;
  double percent2;
  /** Creates a new SerializerCommand. */
  public SerializerCommand(Serializer serializer,double  percent1, double  percent2) {
    this.serializer = serializer;
    this.percent1 = percent1;
    this.percent2 = percent2;
    addRequirements(serializer);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    serializer.runSerMotors(percent1, percent2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    serializer.runSerMotors(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
