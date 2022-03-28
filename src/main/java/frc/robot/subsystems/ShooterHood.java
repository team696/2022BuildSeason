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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterHood extends SubsystemBase {
  public Servo leftActuator;
 public  Servo rightActuator;
//  public WPI_TalonSRX hoodMotor;
 public JohnsonPlg plgEncoder;
 private double hoodSpeed;
 public CANSparkMax hoodMotor;
 public RelativeEncoder encoder;
//  public SparkMaxPIDController controller;

 public PIDController hoodPID; /*MIGHT USE INTAGRATED TALON SRX PID FOR POSITION CONTROL*/

  /** Creates a new ShooterHood. */
  public ShooterHood() {
    leftActuator = new Servo(6);
    rightActuator = new Servo(7);
    hoodMotor = new CANSparkMax(55, MotorType.kBrushed);
    // controller = hoodMotor.getPIDController();
    
    
    // hoodMotor = new WPI_TalonSRX(55);
    // plgEncoder = new JohnsonPlg(8, 9);

    hoodPID = new PIDController(Constants.Shooter.kP, Constants.Shooter.kI, Constants.Shooter.kD); 
    hoodPID.setTolerance(Constants.Shooter.kTolerance);

    hoodMotor.restoreFactoryDefaults();
    hoodMotor.setIdleMode(IdleMode.kBrake);
    hoodMotor.setSoftLimit(SoftLimitDirection.kForward, 1.3f);
    hoodMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 60000);
    hoodMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 60000);
    

    
//  controller.setFeedbackDevice(encoder);
//  controller.setP(Constants.Shooter.kP);
//  controller.setI(Constants.Shooter.kI);
//  controller.setD(Constants.Shooter.kD);
    encoder = hoodMotor.getEncoder(Type.kQuadrature, 8192);
    
    // throughBore = hoodMotor.getEncoder(Type.kQuadrature, 8192);



    // hoodMotor.configFactoryDefault();
    // hoodMotor.setNeutralMode(NeutralMode.Brake);
    // hoodMotor.configNominalOutputForward(0);
    // hoodMotor.configNominalOutputReverse(0);
    // hoodMotor.configPeakOutputForward(Constants.Shooter.hoodPeakOutput);
    // hoodMotor.configPeakOutputReverse(-Constants.Shooter.hoodPeakOutput);
    // hoodMotor.configForwardSoftLimitThreshold(Constants.Shooter.hoodLimitFor);
    // hoodMotor.configReverseSoftLimitThreshold(Constants.Shooter.hoodLimitRev);
    // hoodMotor.configForwardSoftLimitEnable(Constants.Shooter.hoodSoftLimitForward);
    // hoodMotor.configReverseSoftLimitEnable(Constants.Shooter.hoodSoftLimitReverse);
    

    


    // leftActuator.setAngle(0);
    // rightActuator.setAngle(0);
resetEncoderPos();
  }
/** 
 * Method used for controlling the linear actuators.
 * @param position Desired position in angle (about 50 - 130).
 */
  public void moveActuators(double position){
    // if(position<0){
    //   position = 0;
    // }
    // if(position>360){
    //   position = 360;
    // }

    leftActuator.setAngle(position);
    rightActuator.setAngle(position);
    System.out.println("Imhere" + position);
  }

  public void setHoodPos(double pos){
    // hoodMotor.set(hoodPID.calculate(plgEncoder.get(), pos));
    hoodSpeed =  hoodPID.calculate(encoder.getPosition(), pos);
    // if(encoder.getPosition()>1.3){

    //   if(hoodSpeed > 0){
    //     hoodMotor.set(0);
    //   }
    //   else{
    //     hoodMotor.set(hoodSpeed);

    //   }
    // }
    // else{
    hoodMotor.set(hoodSpeed);
    // }

    System.out.println(hoodSpeed);
    // controller.setReference(pos, ControlType.kPosition);
    // hoodMotor.set(TalonSRXControlMode.PercentOutput, hoodSpeed);
  }

  public double getHoodSpeed(){
return 2;
  }

  // public void setPLGEncoder(int pos ){
  //   plgEncoder.set(pos);
  // }

  public double getEncoderPos(){
return encoder.getPosition();  
}
  public void setEncoder(double pos){
encoder.setPosition(pos); 
 }

 public void resetEncoderPos(){
   encoder.setPosition(0);
 }

  public void runHoodMotor(double percent){

    if( encoder.getPosition() > 1.3){
      if(percent > 0){
              hoodMotor.set(0);

      }
      else{
        hoodMotor.set( percent);

  }
    }
    else{
          hoodMotor.set( percent);

    }
  }

  
/**
 * 
 * @return the position of the linear actuators in angle.
 */
  public double servoPosition(){
return leftActuator.getAngle();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
