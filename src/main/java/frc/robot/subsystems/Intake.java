// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  WPI_TalonFX intakeMotor;

  Solenoid intakeSolenoid;

  /** Creates a new Intake. */
  public Intake() {

    intakeMotor = new WPI_TalonFX(50);
    intakeMotor.configFactoryDefault();

    intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 4);
    intakeMotor.setStatusFramePeriod(21, 1000);
    intakeMotor.setStatusFramePeriod(1, 255);
    intakeMotor.setStatusFramePeriod(3, 255);
    intakeMotor.setStatusFramePeriod(4, 255);
    intakeMotor.setStatusFramePeriod(8, 255);
    intakeMotor.setStatusFramePeriod(10 , 255);
    intakeMotor.setStatusFramePeriod(12 , 255);
    



  }
/** Runs the intake motors off percent output
 *  @param power double for the speed of the motor
 * */ 
  public void runIntake(double power){
    intakeMotor.set(TalonFXControlMode.PercentOutput, power);
  }

  /** Deploys the intake pneumatics
    @param state boolean for deployed or stowed
    */
  public void deployIntake(boolean state){
    intakeSolenoid.set(state);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
