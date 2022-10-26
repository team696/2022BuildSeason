// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
 public Talon intakeMotor;
 Solenoid intakeSolenoid;
 
  public Intake() {

    intakeMotor = new Talon( 0);
    
    intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 4);
    
  }
/** Runs the intake motors off percent output
 *  @param power double for the speed of the motor
 * */ 
  public void runIntake(double power){
    intakeMotor.set( power);
  }

  /** Deploys the intake pneumatics
    @param state boolean for deployed or stowed
    */
  public void deployIntake(boolean state){
    intakeSolenoid.set(state);
  }


  @Override
  public void periodic() {
  }
}
