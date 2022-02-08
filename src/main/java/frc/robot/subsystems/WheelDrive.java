// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import java.io.Console;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
// import com.revrobotics.CANEncoder;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class WheelDrive extends SubsystemBase {


  private WPI_TalonFX angleMotor;
  private WPI_TalonFX speedMotor;
  // private PIDController pidController;
  private final double  MAX_VOLTS = 4.95;
  public double motorSpeed;
  public double inpangle;
  // public FeedbackDevice pleaseWork;
  public CANCoder wheelEncoder;
  /** Creates a new WheelDrive. */
  public WheelDrive(int angleMotor, int speedMotor, int encoder) {

    this.angleMotor = new WPI_TalonFX(angleMotor);
    this.speedMotor = new WPI_TalonFX(speedMotor);
    
    this.angleMotor.configFactoryDefault();
    this.angleMotor.configRemoteFeedbackFilter(encoder, RemoteSensorSource.CANCoder, 0);
    this.angleMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.RemoteSensor0, 1, 30);

    this.angleMotor.setNeutralMode(NeutralMode.Coast);
    this.angleMotor.configPeakOutputForward(1);
    this.angleMotor.configPeakOutputReverse(-1);

    
    this.angleMotor.config_kF(1, 1, 30);
    this.angleMotor.config_kP(1, 1, 30);
    this.angleMotor.config_kI(1, 0.1, 30);
    this.angleMotor.config_kD(1, 0.0, 30);
    
wheelEncoder = new CANCoder(encoder);
wheelEncoder.configFactoryDefault();
wheelEncoder.setPosition(0);


// pidController = new PIDController(0.25, 0.0, 0);
// pidController.enableContinuousInput(-1, 1);


    


    // pidController = new PIDController(kp, ki, kd, period)
  }

  public void drive(double speed, double angle){
// speedMotor.set(speed);

// double setpoint = angle * (MAX_VOLTS * 0.5) + (MAX_VOLTS*0.5);
// if ( setpoint<0){
//   setpoint = MAX_VOLTS+setpoint;
// }
// if(setpoint> MAX_VOLTS) {
//   setpoint = setpoint- MAX_VOLTS;
// }
angleMotor.set(ControlMode.Position, angle);
}
//  double wheelpos = wheelEncoder.getPosition();
// //  motorSpeed = pidController.calculate(wheelpos, setpoint);
//  System.out.print("Setpoint: " + setpoint);
//  System.out.print(", Speed: " + motorSpeed);


//  System.out.println(", Current Angle: " + wheelpos);
 


// angleMotor.set(motorSpeed);
// speedMotor.set(speed);

//System.out.println(wheelEncoder.getPosition());


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
