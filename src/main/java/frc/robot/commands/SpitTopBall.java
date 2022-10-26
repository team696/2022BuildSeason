// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DIOSub;
import frc.robot.subsystems.Serializer;
import frc.robot.subsystems.Shooter;

public class SpitTopBall extends CommandBase {
  Serializer serializer;
  Shooter shooter;
  int step;
  boolean done;
  double timer;
  /** Runs the shooter at a low speed, spits out the top ball in the serializer and serializes the bottom ball up to the beam break. */
  public SpitTopBall(Serializer serializer, Shooter shooter) {
    this.serializer = serializer;
    this.shooter = shooter;
    addRequirements(serializer, shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    done = false;
    step = 0;
    timer = 0;
  }

  @Override
  public void execute() {
    switch(step){
      case 0:
      if(!serializer.beamBreak.get()){
      shooter.setShooterPercent(0.2);;
      serializer.runSerMotors(0.2, -0.2);

      }
      else{
        step = 1;
      }
      break;

      case 1:
      if(serializer.beamBreak.get()){
      shooter.setShooterPercent(0.2);
      serializer.runSerMotors(0.2, -0.2);
      }
      else{
        step = 2;

      }
      break;
      case 2:

      timer++;
      if(timer <20){
      shooter.setShooterPercent(0.2);
      serializer.runSerMotors(0, 0);
      }
      else{
      shooter.setShooterPercent(0);
      serializer.runSerMotors(0, 0);
            done = true;

      }
      break;
         


    }
    
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setShooter(0);
      serializer.runSerMotors(0, 0);
  }

  @Override
  public boolean isFinished() {
    return done;
  }
}
