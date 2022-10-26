// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
  public DoubleSolenoid sol1;
  public DoubleSolenoid sol2;
  public Compressor compressor;
  public PneumaticsControlModule module;
  public enum LatchStates{
    DOUBLE_LATCHES, SINGLE_LATCHES

  }
  public LatchStates latchState = LatchStates.SINGLE_LATCHES;
  public Pneumatics() {
    sol2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 3);
    sol1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 2);

    module = new PneumaticsControlModule(1);
    module.clearAllStickyFaults();
    
  }

/** Method used to control the climber pneumatics
 * @param state which latches you want to move
 * @param value which direction you want to move them
 * */ 
  public void autoPneumatics(LatchStates state, Value value){
    switch(state) {

      case SINGLE_LATCHES:

      movePneumatics2(value);

      break;

      case DOUBLE_LATCHES:

      movePeumatics1(value);
      
      break;

    }

  }
    
  public void movePeumatics1(Value value){
    sol1.set(value);
  }

  public void movePneumatics2(Value value){
    sol2.set(value);
  }
  @Override
  public void periodic() {
  }
}
