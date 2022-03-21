// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterHood extends SubsystemBase {
  public Servo leftActuator;
 public  Servo rightActuator;
 public WPI_TalonSRX hoodMotor;

 public PIDController hoodPID; /*MIGHT USE INTAGRATED TALON SRX PID FOR POSITION CONTROL*/

  /** Creates a new ShooterHood. */
  public ShooterHood() {
    leftActuator = new Servo(6);
    rightActuator = new Servo(7);
    // hoodMotor = new WPI_TalonSRX(50);

    hoodPID = new PIDController(Constants.Shooter.kP, Constants.Shooter.kI, Constants.Shooter.kD); 
    hoodPID.setTolerance(Constants.Shooter.kTolerance);


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
