// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSub extends SubsystemBase {
  WPI_TalonFX lClimberMotor;
  WPI_TalonFX rClimberMotor;
  /** Creates a new ClimberSub. */
  public ClimberSub() {

    lClimberMotor = new WPI_TalonFX(21);
    rClimberMotor = new WPI_TalonFX(20);

    rClimberMotor.configFactoryDefault();
    rClimberMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 1, 30);
    rClimberMotor.setSensorPhase(true);
    rClimberMotor.setInverted(false);
    rClimberMotor.config_kF(1, 1.0, 30);
    rClimberMotor.config_kP(1, 1.0, 30);
    rClimberMotor.config_kI(1, 0, 30);
    rClimberMotor.config_kD(1, 0, 30);

    lClimberMotor.configFactoryDefault();
    // lClimberMotor.follow(rClimberMotor);
    lClimberMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 1, 30);
    lClimberMotor.setSensorPhase(false);
    lClimberMotor.setInverted(true);
    lClimberMotor.config_kF(1, 1.0, 30);
    lClimberMotor.config_kP(1, 1.0, 30);
    lClimberMotor.config_kI(1, 0, 30);
    lClimberMotor.config_kD(1, 0, 30);

  }

  public void moveClimber(double percent){
    rClimberMotor.set(TalonFXControlMode.PercentOutput, percent);
    lClimberMotor.set(TalonFXControlMode.PercentOutput, percent);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
