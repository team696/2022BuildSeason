// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DIOSub;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Serializer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.TrajectoryTable;

public class AutoShoot extends CommandBase {
  Limelight limelight;
  Shooter shooter;
  Serializer serializer;
  ShooterHood shooterHood;
  boolean finished;
  TrajectoryTable trajectoryTable;
  

  boolean locked;
  int step;
  boolean done;
  double timer;
  double overallTimer;
  double shootTimer;
  double speed;
  double last_angle;

  /** Sets the angle and speed of the shooter for the distance, then fires 2 balls after a delay. */
  public AutoShoot(Limelight limelight, Shooter shooter, Serializer serializer, ShooterHood shooterHood, TrajectoryTable trajectoryTable) {
    this.limelight = limelight;
    this.shooter = shooter;
    this.serializer = serializer;
    this.shooterHood = shooterHood;
    this.trajectoryTable = trajectoryTable;
    ;

    addRequirements(limelight, shooter, serializer, shooterHood ,trajectoryTable);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shootTimer = 0;
    overallTimer = 0;
    done = false;
    step = 0;
    timer = 0;
    finished = false;
    limelight.setLights(3);
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

    overallTimer++;
    locked = limelight.crosshairOnTarget();
    speed = shooter.getRequiredShootSpeed();
       /* ====================== SETS SHOOTER AND HOOD ======================*/

        shooter.setShooter(shooter.rpmToTalonFX(speed ));

        // if(distance <21){
        //   angle  = trajectoryTable.distanceToHoodAngle[distance];
        //   limelight.setLights(3);
        //   shooterHood.moveActuators(angle);
        //     last_angle = angle;
        //   }
        //   else{
        //     shooterHood.moveActuators(last_angle);
        //   }
        // double  testEquation = limelight.getDistance()/12 * 0.062;
        double testEquation =  (14.1714 * Math.sqrt((limelight.getDistance()/12) -5))/46.8497;

 if(limelightDistance < 5.5){
      limelight.pipeline(0);
    }
    else{
      limelight.pipeline(1);
    }
    if(distance <21 && distance > 5){
    // angle  = trajectoryTable.distanceToHoodAngle[distance];
    angle = testEquation;
    limelight.setLights(3);
    shooterHood.setHoodPos(angle);
      last_angle = angle;
    }
    else{
      shooterHood.setHoodPos(last_angle);
    }
   
          shootTimer++;
        /* ====================== FIRE COMMAND ======================*/

  if(shootTimer > 40){

      
   
    // if( shooter.getShooterRPM() > (speed )){
      switch(step){
        case 0:
        serializer.runSerMotors(0.2, -0.4);
         if(!serializer.beamBreak.get()){
          serializer.runSerMotors(0.2, -0.4);
          overallTimer = 0;
          }
        else{
        step = 1;
        }       
        break;                                                    /* ITS UGLY AF I KNOW  */
        case 1:
        timer++;
        if(timer < 60){                                                   /* 0.4, -0.6 */
       if(serializer.beamBreak.get()){
       serializer.runSerMotors(0.2, -0.4);
       }
       else{
         step = 2;
         timer = 0;
         overallTimer = 0;
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
          overallTimer = 0;
        }
        break;
        case 3:
        
        timer++;
        if(timer < 20){
          serializer.runSerMotors(0.2, -0.4);
        }
        else{
          serializer.runSerMotors(0, 0.0);
          done = true;
        }
      } 
     }
     else{
       serializer.runSerMotors(0, 0);
     }

      /* ====================== TIMER ======================*/

     if (overallTimer > 300){
       done = true;
     }

      /* ====================== SWITCHES PIPELINES ======================*/

 
  if(limelightDistance < 5.5){
       limelight.pipeline(0);
     }
     else{
       limelight.pipeline(1);
     }
    
}



    

     
  // }

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
