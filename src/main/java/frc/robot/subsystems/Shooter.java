// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
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
    leftShooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    leftShooterMotor.setSensorPhase(true );
     leftShooterMotor.setNeutralMode(NeutralMode.Coast);
    leftShooterMotor.configPeakOutputForward(0.9);
    leftShooterMotor.configPeakOutputReverse(-0.9);
    leftShooterMotor.configNominalOutputForward(0);
    leftShooterMotor.configNominalOutputReverse(0);
    leftShooterMotor.configClosedloopRamp(0.5);
    leftShooterMotor.config_kP(0, 0.1);
    leftShooterMotor.config_kI(0, 0.0);
    leftShooterMotor.config_kD(0, 0.0);
    leftShooterMotor.config_kF(0, 0.06);
    leftShooterMotor.configAllowableClosedloopError(0, 10);
        // leftShooterMotor.setInverted(false);

    rightShooterMotor.configFactoryDefault();
    rightShooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 1, 10);
    rightShooterMotor.setSensorPhase(true );
    rightShooterMotor.setNeutralMode(NeutralMode.Coast);
    rightShooterMotor.configPeakOutputForward(0.9 );
    rightShooterMotor.configPeakOutputReverse(-0.9);
    rightShooterMotor.configNominalOutputForward(0);
    rightShooterMotor.configNominalOutputReverse(0);
    
    // rightShooterMotor.config_kP(1, 0.6);
    // rightShooterMotor.config_kI(1, 0.1);
    // rightShooterMotor.config_kD(1, 0.1);
    // rightShooterMotor.config_kF(1, .5);

    // leftShooterMotor.config_kP(1, 0.6);
    // leftShooterMotor.config_kI(1, 0.1);
    // leftShooterMotor.config_kD(1, 0.1);
    // leftShooterMotor.config_kF(1, .5);
  }
  
  public double rpmToTalonFX(double rpm){
    double retval = rpm / 0.667 / 60 / 10 * 2048;
    return retval;
  }

  public double talonFXtoRPM(double speed){     // talonFX = encoder ticks per 100ms
    double rpm = speed/2048 * 10 * 60 * 0.667;  //  (rotations per 1/10s) * encoder updates per second * rps to rpm * gear ratio
    return rpm;
  }


  public double getShooterRPM(){
    return talonFXtoRPM(leftShooterMotor.getSelectedSensorVelocity());

  }

  public void setShooterPercent(double percent){
    leftShooterMotor.set(TalonFXControlMode.PercentOutput, percent);
  }

  



  public void setShooter(double speed){
    System.out.println("speed: " + speed);
    leftShooterMotor.set(TalonFXControlMode.Velocity, speed);
  }
  public void shooterOff(){
    leftShooterMotor.disable();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
