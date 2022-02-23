// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DIOSub;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Pneumatics.LatchStates;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Constants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoClimbSequenceNew extends CommandBase {
  Climber climber;
  Pneumatics pneumatics;
  DIOSub dioSub;
  boolean[] sensor;
  int counter = 0;
  int stage = 0; //State 0 is init, 1 is latched and climbing to high, 2 is waiting for mid bar successful release, 3 is high to travese climb, 4 is waiting for traverse latch, 5 is release + hold, 6 is complete climb
  
  /** Creates a new AutoClimbeSequence. */
  public AutoClimbSequenceNew(Climber climber, Pneumatics pneumatics, DIOSub dioSub) {
    this.climber = climber;
    this.pneumatics = pneumatics;
    this.dioSub = dioSub;

  }
    @Override
    public void initialize() {
    }
    
    @Override
    public void execute() {

      switch (stage){
        case 0: //initialize robot for climbing
          if ((DIOSub.DH_L_B && DIOSub.DH_R_B) || (DIOSub.DH_L_T && DIOSub.DH_R_T)){ //Either top or bottom must be fully seated on the bar 
            pneumatics.autoPneumatics(LatchStates.DOUBLE_LATCHES, Value.kForward); //Ensures the latches are closed on the bar
            stage = 1; //moves to stage 1
          }
        case 1:
          climber.moveClimber(Constants.Climber.CLIMB_SPEED); //Begin climbing towards high
          if (DIOSub.SH_L || DIOSub.SH_R){ //wait until any of the single hands detect a bar
            pneumatics.autoPneumatics(LatchStates.DOUBLE_LATCHES, Value.kReverse); //immediately release the double hand hooks on mid bar
            stage = 2; //moves to stage 2, etc
          }
        case 2:
          climber.moveClimber(0);
          if (counter > 20){ //approx 400ms delay with the climber off for pneumatics to fully retract
            counter = 0;
            stage = 3;
          }
          else{
            counter++;
          }
        case 3:
          climber.moveClimber(Constants.Climber.CLIMB_SPEED); //continue towards traverse climb
          if (DIOSub.DH_L_B || DIOSub.DH_R_B || DIOSub.DH_L_T || DIOSub.DH_R_T){
            pneumatics.autoPneumatics(LatchStates.DOUBLE_LATCHES, Value.kForward); //immediately command latches on traverse bar once a bar has been detected
            stage = 4;
          }
        case 4:
          climber.moveClimber(Constants.Climber.HOLD_SPEED); //hold the robot on the traverse while the double latches extend for 400ms
          if (counter > 20){ //approx 400ms
            stage = 5;
            counter = 0;
          }
          else{
            counter++;
          }
        case 5:
          if ((DIOSub.DH_L_B && DIOSub.DH_R_B) || (DIOSub.DH_L_T && DIOSub.DH_R_T)){ //SMART: only release if a bar is detected in the double hands
            pneumatics.autoPneumatics(LatchStates.SINGLE_LATCHES, Value.kReverse); //Release the single hand latches
            climber.moveClimber(-Constants.Climber.HOLD_SPEED); //hold for 400ms while the single hand hooks retract
            if (counter > 20){ //approx 400ms
              stage = 6;
              counter = 0;
            }
            else{
              counter++;
            }
          }
          else{
            end(true);
          }
        case 6:
        climber.moveClimber(0);

      }

    }

    @Override
    public void end(boolean interrupted) {
      climber.moveClimber(0);

    }

    @Override
    public boolean isFinished() {
      return false;
    }
    
}
