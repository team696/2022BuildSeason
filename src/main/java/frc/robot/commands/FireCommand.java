// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Serializer;

public class FireCommand extends CommandBase {
  Serializer serializer;
  int step;
  boolean done;
  double timer;
  /** Fires one ball and then uses the beam break to delay the second ball. */
  public FireCommand(Serializer serializer ) {
    this.serializer = serializer;
    addRequirements(serializer );
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
    step = 0;
    timer = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(step){
      case 0:
       if(!serializer.beamBreak.get()){
        serializer.runSerMotors(0.2, -0.4);

        }
      else{
      step = 1;
      }       
      break;

      case 1:
      timer++;
      if(timer < 60){
      
     if(serializer.beamBreak.get()){
     serializer.runSerMotors(0.2, -0.4);
     }
     else{
       step = 2;
       timer = 0;
     }
   }
    else{
      done = true;
    }
                
      break;
      case 2:

      timer++;
      if(timer <5){
      serializer.runSerMotors(0.0, 0.0);
      }
      else{
        timer = 0;
        step = 3;
      }
      break;
      case 3:
      timer++;
      if(timer < 20){
        serializer.runSerMotors(0.2, -0.4);
      }
      else{
        serializer.runSerMotors(0, 0);
        done = true;
      }
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    serializer.runSerMotors(0, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
