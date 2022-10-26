// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterHood extends SubsystemBase {

 public JohnsonPlg plgEncoder;
 private double hoodSpeed;
 public CANSparkMax hoodMotor;
 public RelativeEncoder encoder;

 public PIDController hoodPID; 

  public ShooterHood() {
    hoodMotor = new CANSparkMax(55, MotorType.kBrushed);
    
    hoodPID = new PIDController(Constants.Shooter.kP, Constants.Shooter.kI, Constants.Shooter.kD); 
    hoodPID.setTolerance(Constants.Shooter.kTolerance);
  
    hoodMotor.restoreFactoryDefaults();
    hoodMotor.setIdleMode(IdleMode.kBrake);
    hoodMotor.setSoftLimit(SoftLimitDirection.kForward, 1.3f);
    hoodMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 60000);
    hoodMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 60000);
    hoodMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 60000);
  
    encoder = hoodMotor.getEncoder(Type.kQuadrature, 8192);
   
    resetEncoderPos();


  }

  public void setHoodPos(double pos){
    hoodSpeed =  hoodPID.calculate(encoder.getPosition(), pos);
    if(encoder.getPosition()>17){
    setEncoder(16.8);
    }

  hoodMotor.set(hoodSpeed);
    

  }

  public double getHoodSpeed(){
    return 2;


  }

  public double getEncoderPos(){
    return encoder.getPosition(); 
    
    
  }

/**
 * Sets the through bore encoder on the hood to a given position.
 * @param pos
 */
  public void setEncoder(double pos){
    encoder.setPosition(pos); 


  }

/**Sets the through bore encoder on the hood to zero. */
 public void resetEncoderPos(){
  encoder.setPosition(0);


 }

 /**
  * Runs the Johnson PLG on the hood with the given percent.
  * @param percent Desired percent output.
  */
  public void runHoodMotor(double percent){

    if( encoder.getPosition() > 17){
      if(percent > 0){
              hoodMotor.set(0);

      }
      else{
        hoodMotor.set( percent);

      }
    } else {
          hoodMotor.set( percent);

    }


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
