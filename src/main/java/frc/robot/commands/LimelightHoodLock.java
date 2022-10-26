// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.LEDSub;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.TrajectoryTable;
import frc.robot.Constants;
import frc.robot.commands.TeleopSwerve;

public class LimelightHoodLock extends CommandBase {
  Limelight limelight;
  TrajectoryTable trajectoryTable;
  ShooterHood shooterHood;
  LEDSub ledSub;
  int mode;
  double last_angle;
  boolean idle;
    //double midpart = Math.PI/2 - Math.atan( (Constants.Shooter.height + Math.sqrt( Constants.Shooter.height * Constants.Shooter.height + limelightDistance * limelightDistance )) / limelightDistance);
    //double iniAngle = 4.8185346 * (midpart - 0.2268928);
  /** Moves the hood based on the distance gotten from the limelight. */
  public LimelightHoodLock(Limelight limelight, TrajectoryTable trajectoryTable,ShooterHood shooterHood, int mode, LEDSub ledSub, boolean idle) {
    this.limelight = limelight;
    this.trajectoryTable = trajectoryTable;
    this.shooterHood = shooterHood;
    this.mode = mode;
    this.ledSub = ledSub;
    this.idle = idle;
    addRequirements(limelight, trajectoryTable, shooterHood, ledSub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    last_angle = 0.6;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   boolean locked = limelight.crosshairOnTarget();
    int distance;
    double limelightDistance;
    double angle;
    limelightDistance = limelight.getDistance()/12;

    distance = (int)Math.round(limelightDistance);


    

 
   
if (idle){
  ledSub.setRightLEDs(50, 50, 50);
  shooterHood.setHoodPos(0);
} else {
      //if (limelightDistance < 5.5) {
      //  limelight.pipeline(0);
      //} else{
      //  limelight.pipeline(1);
      //}
      limelight.pipeline(2);
      if(distance <25 && distance > 5){
        angle =trajectoryTable.distanceToHoodAngle[distance] ;
        limelight.setLights(mode);
        shooterHood.setHoodPos(angle);
        last_angle = angle;
      } else{
        shooterHood.setHoodPos(last_angle);
      }
    if (locked) {
      ledSub.setRightLEDs(0, 255, 0);
    } else {
      ledSub.setRightLEDs(255, 0, 0);

    }
  }   
}
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    limelight.setLights(1);
    ledSub.setRightLEDsHSV(0, 0, 50);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
