// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  WPI_TalonFX leftShooterMotor;
  WPI_TalonFX rightShooterMotor;

  /** Creates a new Shooter. */
  public Shooter() {


    leftShooterMotor = new WPI_TalonFX(40);
    rightShooterMotor = new WPI_TalonFX(41);

    leftShooterMotor.configFactoryDefault();
    leftShooterMotor.setSensorPhase(true );
    leftShooterMotor.setInverted(false);
    leftShooterMotor.setNeutralMode(NeutralMode.Coast);
    leftShooterMotor.configPeakOutputForward(1);
    leftShooterMotor.configPeakOutputReverse(-1);
    
    leftShooterMotor.config_kP(1, 1.0);
    leftShooterMotor.config_kI(1, 0);
    leftShooterMotor.config_kD(1, 0);
    leftShooterMotor.config_kF(1, 1);
    
    rightShooterMotor.configFactoryDefault();
    rightShooterMotor.follow(leftShooterMotor);
    rightShooterMotor.setSensorPhase(true );
    rightShooterMotor.setInverted(true );
    rightShooterMotor.setNeutralMode(NeutralMode.Coast);
    rightShooterMotor.configPeakOutputForward(1);
    rightShooterMotor.configPeakOutputReverse(-1);
    
    rightShooterMotor.config_kP(1, 1.0);
    rightShooterMotor.config_kI(1, 0);
    rightShooterMotor.config_kD(1, 0);
    rightShooterMotor.config_kF(1, 1);
  }

  public double rpmToTalonFX(double rpm){
    double retval = ((rpm/600)*(2048/0.66));
    return retval;
  }

  public double talonFXtoRPM(double speed){
    double rpm = speed/2048 * 600 * 0.66;
    return rpm;
  }


  public double getShooterRPM(){
    return talonFXtoRPM(leftShooterMotor.getSelectedSensorVelocity());

  }



  public void setShooter(double speed){
    leftShooterMotor.set(TalonFXControlMode.PercentOutput, speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
