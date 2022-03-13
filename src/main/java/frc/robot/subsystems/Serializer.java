// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Serializer extends SubsystemBase {
  TalonFX leftSerMotor;
  TalonFX rightSerMotor;
  public DigitalInput beamBreak;
  Limelight limelight;
  TrajectoryTable trajectoryTable;

  /** Creates a new Serializer. */
  public Serializer() {
    limelight = new Limelight();
    trajectoryTable = new TrajectoryTable();
    beamBreak = new DigitalInput(6);
    leftSerMotor = new TalonFX(Constants.Serializer.leftSerMotorport);
    rightSerMotor = new TalonFX(Constants.Serializer.rightSerMotor);
    
    leftSerMotor.configFactoryDefault();
    leftSerMotor.setSensorPhase(Constants.Serializer.serializerMotorSensorPhase);
    leftSerMotor.setInverted(Constants.Serializer.serializerMotorInverted);
    leftSerMotor.setNeutralMode(NeutralMode.Brake);
    leftSerMotor.configPeakOutputForward(Constants.Serializer.peakOutput);
    leftSerMotor.configPeakOutputReverse(-Constants.Serializer.peakOutput);
    leftSerMotor.configForwardSoftLimitEnable(false);
    leftSerMotor.configReverseSoftLimitEnable(false);
    

    rightSerMotor.configFactoryDefault();
    rightSerMotor.setSensorPhase(Constants.Serializer.serializerMotorSensorPhase);
    rightSerMotor.setInverted(Constants.Serializer.serializerMotorInverted);
    rightSerMotor.setNeutralMode(NeutralMode.Brake);
    rightSerMotor.configPeakOutputForward(Constants.Serializer.peakOutput);
    rightSerMotor.configPeakOutputReverse(-Constants.Serializer.peakOutput);
    rightSerMotor.configForwardSoftLimitEnable(false);
    rightSerMotor.configReverseSoftLimitEnable(false);
  }

  /**
   * Method for running the serializer motors.
   * @param percent percent output for the top motor.
   * @param percent2 percent output for the bottom  motor.
   */
    public void runSerMotors(double percent, double percent2){
      leftSerMotor.set(TalonFXControlMode.PercentOutput, percent);
      rightSerMotor.set(TalonFXControlMode.PercentOutput, percent2);
    }

    
  public double getRequiredShootSpeed(){
    int distance;
    double limelightDistance;
    double speed;
    // double speed;
    limelightDistance = limelight.getDistance()/12;
    distance = (int)Math.round(limelightDistance);

    if(distance <21){
    speed  = trajectoryTable.distanceToShooterSpeed[distance];
    // speed = trajectoryTable.distanceToShooterSpeed[distance];
    // limelight.setLights(mode);
    // shooterHood.moveActuators(angle);
    return speed + 100;
    }
    else{
      return 3100;
    }
    
  }

  }
  

  // @Override
  // public void periodic() {
  //   // This method will be called once per scheduler run
  // }


