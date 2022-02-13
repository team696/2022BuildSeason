// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.fasterxml.jackson.annotation.JacksonInject.Value;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Climber extends SubsystemBase {
  //  private RobotContainer robotContainer;
  public WPI_TalonFX lClimberMotor;
  public WPI_TalonFX rClimberMotor;

   public DoubleSolenoid sol1;
  public DoubleSolenoid sol2;
  public Compressor compressor;

  double climberDirection;
  /** Creates a new Climber. */
  public Climber() {
    
    lClimberMotor = new WPI_TalonFX(21);
    rClimberMotor = new WPI_TalonFX(20);

     rClimberMotor.configFactoryDefault();
    // rClimberMotor.follow(lClimberMotor);
    // rClimberMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 1, 30);
     rClimberMotor.setNeutralMode(NeutralMode.Coast);
    // rClimberMotor.setSensorPhase(true);
    // // rClimberMotor.setInverted(true);
    // rClimberMotor.config_kF(1, 1.0, 30);
    // rClimberMotor.config_kP(1, 1.0, 30);
    // rClimberMotor.config_kI(1, 0, 30);
    // rClimberMotor.config_kD(1, 0, 30);

    lClimberMotor.configFactoryDefault();
    // lClimberMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 1, 30);
    lClimberMotor.setNeutralMode(NeutralMode.Coast );
    // lClimberMotor.setSensorPhase(true);
    // lClimberMotor.setInverted(true);
    // lClimberMotor.config_kF(1, 1.0, 30);
    // lClimberMotor.config_kP(1, 1.0, 30);
    // lClimberMotor.config_kI(1, 0, 30);
    // lClimberMotor.config_kD(1, 0, 30);

    sol2 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 1, 3);
    sol1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 2);
    compressor = new Compressor(1, PneumaticsModuleType.REVPH);

    compressor.enableAnalog(90, 120);

  }

  // public double getClimberAxis(){
  //   return robotContainer.controlPanel.getRawAxis(0);
  // }

  public void movePeumatics1(Value value){
    sol1.set(value);
  }

  public void movePneumatics2(Value value){
    sol2.set(value);
  }
  
  public void moveClimber(double percent){
    // rClimberMotor.set(TalonFXControlMode.PercentOutput, percent);
    lClimberMotor.set(TalonFXControlMode.PercentOutput, percent);
    rClimberMotor.set(TalonFXControlMode.PercentOutput, percent);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
