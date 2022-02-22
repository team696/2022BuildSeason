// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  WPI_TalonFX intakeMotor;

  DoubleSolenoid intakeSolenoid;

  /** Creates a new Intake. */
  public Intake() {

    intakeMotor = new WPI_TalonFX(50);

    intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 4, 5);


  }

  public void runIntake(double power){
    intakeMotor.set(TalonFXControlMode.PercentOutput, power);
  }

  public void deployIntake(Value value){
    intakeSolenoid.set(value);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
