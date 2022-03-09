// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;

public class FlashBangCommand extends CommandBase {
  Limelight limelight;
  double msTimer;
  /** DEFINETELY DOES NOT STROBE THE LIMELIGHTS IN ORDER TO FLASHBANG OTHER TEAMS */
  public FlashBangCommand(Limelight limelight) {
    this.limelight = limelight;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    msTimer = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    msTimer++;
    if(msTimer == 2){
      limelight.setLights(1);
      msTimer = 0;
    }
    else{
      limelight.setLights(3);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limelight.setLights(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
