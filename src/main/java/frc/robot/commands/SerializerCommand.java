// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DIOSub;
import frc.robot.subsystems.Serializer;
import frc.robot.subsystems.Shooter;

public class SerializerCommand extends CommandBase {
  Serializer serializer;
  double percent1;
  double percent2;
  boolean stoppable;
  /** Runs the serializer motors. Will not move the ball past the beam break unless shooting.
   * 
   * 
   * @param percent1 percent output for top motor.
   * @param percent2 percent outup for bottom motor.
   */
  public SerializerCommand(Serializer serializer, double  percent1, double  percent2 ) {
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

  if(RobotContainer.isShooting ){
    stoppable = false;
  }

  else{
    stoppable = true;
  }
  if(stoppable){
    boolean broken = serializer.beamBreak.get();
    if(broken){ 
      serializer.runSerMotors(percent1, percent2);
    }
    else{

      serializer.runSerMotors(0, 0);
    }
  }
  else{
    if (percent1 != 0){
      serializer.runSerMotors(Constants.Serializer.SHOOT_SPEED, -Constants.Serializer.SHOOT_SPEED);
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
    return false;
  }
}
