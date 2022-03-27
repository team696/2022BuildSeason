// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterHood;

public class ShooterHoodCommand extends CommandBase {
  ShooterHood shooterHood;
  double    position;
  double last_hood_axis;
  // double timer;
  /** Open loop hood control. */
  public ShooterHoodCommand(ShooterHood shooterHood, double  position) {
    this.shooterHood = shooterHood;
    this.position = position;
    addRequirements(shooterHood);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // timer = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // shooterHood.setHoodPos(position);
    // shooterHood.runHoodMotor(RobotContainer.controlPanel.getRawAxis(Constants.Shooter.hoodAxis));

    // timer++;  
    //     double hood_axis = -Math.round(RobotContainer.controlPanel.getRawAxis(Constants.Shooter.hoodAxis)*128*1.5*0.5);

    // if(RobotContainer.controlPanel.getRawButton(Constants.Shooter.hoodAutoButton)){
    //   // shooterHood.moveActuators(100);
    //   shooterHood.setHoodPos(100);

      

    // }
    // else{
    //           // shooterHood.moveActuators(/* shooterHood.servoPosition() + */ hood_axis + 100);
    //           shooterHood.setHoodPos(hood_axis);


      // double delta = hood_axis - last_hood_axis;
    
      // SmartDashboard.putNumber("Hood Axis", Math.round(RobotContainer.controlPanel.getRawAxis(Constants.Shooter.hoodAxis)*128*0.5));

      // if (delta + shooterHood.servoPosition() <= 150 && delta + shooterHood.servoPosition() >= 51){
      
       
      // }
      // last_hood_axis = hood_axis;
  shooterHood.runHoodMotor(RobotContainer.controlPanel.getRawAxis(Constants.Shooter.hoodAxis));
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
