// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SerializerConstants;

public class Serializer extends SubsystemBase {
  TalonFX leftSerMotor;
  TalonFX rightSerMotor;

  /** Creates a new Serializer. */
  public Serializer() {
    leftSerMotor = new TalonFX(SerializerConstants.leftSerMotorport);
    rightSerMotor = new TalonFX(SerializerConstants.rightSerMotor);
    
    leftSerMotor.configFactoryDefault();
    leftSerMotor.setSensorPhase(SerializerConstants.serializerMotorSensorPhase);
    leftSerMotor.setInverted(SerializerConstants.serializerMotorInverted);
    leftSerMotor.setNeutralMode(NeutralMode.Brake);
    leftSerMotor.configPeakOutputForward(SerializerConstants.peakOutput);
    leftSerMotor.configPeakOutputReverse(-SerializerConstants.peakOutput);
    leftSerMotor.configForwardSoftLimitEnable(false);
    leftSerMotor.configReverseSoftLimitEnable(false);

    rightSerMotor.configFactoryDefault();
    rightSerMotor.setSensorPhase(SerializerConstants.serializerMotorSensorPhase);
    rightSerMotor.setInverted(SerializerConstants.serializerMotorInverted);
    rightSerMotor.setNeutralMode(NeutralMode.Brake);
    rightSerMotor.configPeakOutputForward(SerializerConstants.peakOutput);
    rightSerMotor.configPeakOutputReverse(-SerializerConstants.peakOutput);
    rightSerMotor.configForwardSoftLimitEnable(false);
    rightSerMotor.configReverseSoftLimitEnable(false);
  }
    public void runSerMotors(double percent, double percent2){
      leftSerMotor.set(TalonFXControlMode.PercentOutput, percent);
      rightSerMotor.set(TalonFXControlMode.PercentOutput, percent2);
    }

  }
  

  // @Override
  // public void periodic() {
  //   // This method will be called once per scheduler run
  // }


