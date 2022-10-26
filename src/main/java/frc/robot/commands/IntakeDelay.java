// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeDelay extends CommandBase {
  Intake intake;
  double power;
  boolean state;
  double timer;
  /** Runs both the intake pneumatics and motors.
   * 
   * 
   * @param power power of the intake motors.
   * @param state the state of the intake pneumatics.
   */
  public IntakeDelay(Intake intake, double power, boolean state ) {
    this.intake = intake;
    this.power = power;
    this.state = state;
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer++;
    intake.deployIntake(state);

    if(timer > 40){
      intake.runIntake(power);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    

    intake.runIntake(0);
    intake.deployIntake(false);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
