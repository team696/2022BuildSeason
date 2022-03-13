// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.TrajectoryTable;

public class LimelightHoodLock extends CommandBase {
  Limelight limelight;
  TrajectoryTable trajectoryTable;
  ShooterHood shooterHood;
  int mode;
  double last_angle;

  /** Moves the hood based on the distance gotten from the limelight. */
  public LimelightHoodLock(Limelight limelight, TrajectoryTable trajectoryTable,ShooterHood shooterHood, int mode) {
    this.limelight = limelight;
    this.trajectoryTable = trajectoryTable;
    this.shooterHood = shooterHood;
    this.mode = mode;
    addRequirements(limelight, trajectoryTable, shooterHood);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    last_angle = 50;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   
    int distance;
    double limelightDistance;
    double angle;
    limelightDistance = limelight.getDistance()/12;
    distance = (int)Math.round(limelightDistance);

 if(limelightDistance < 5.5){
      limelight.pipeline(0);
    }
    else{
      limelight.pipeline(1);
    }
    if(distance <21){
    angle  = trajectoryTable.distanceToHoodAngle[distance];
    limelight.setLights(mode);
    shooterHood.moveActuators(angle);
      last_angle = angle;
    }
    else{
      shooterHood.moveActuators(last_angle);
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
