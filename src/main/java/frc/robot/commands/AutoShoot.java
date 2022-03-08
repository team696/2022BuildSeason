// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Serializer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;

public class AutoShoot extends CommandBase {
  Limelight limelight;
  Shooter shooter;
  Serializer serializer;
  boolean finished;
  boolean locked;
  int step;
  boolean done;
  double timer;

  /** Creates a new AutoShoot. */
  public AutoShoot(Limelight limelight, Shooter shooter, Serializer serializer) {
    this.limelight = limelight;
    this.shooter = shooter;
    this.serializer = serializer;
    addRequirements(limelight, shooter, serializer);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
    step = 0;
    timer = 0;
    finished = false;
    limelight.setLights(3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    locked = limelight.crosshairOnTarget();
    if(locked){
      
   
    shooter.setShooter(shooter.rpmToTalonFX(shooter.getRequiredShootSpeed()));
    if( shooter.getShooterRPM() >= shooter.getRequiredShootSpeed()){
      switch(step){
        case 0:
         if(!serializer.beamBreak.get()){
          serializer.runSerMotors(0.2, -0.4);
          }
        else{
        step = 1;
        }       
        break;                                                    /* ITS UGLY AF I KNOW  */
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
        // finished = true;
     }
}
    

     
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    serializer.runSerMotors(0, 0);
    shooter.setShooter(0);
    limelight.setLights(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
