// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
 public Talon intakeMotor;
 Solenoid intakeSolenoid;

  // Solenoid intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 4);

  /** Creates a new Intake. */
  public Intake() {

    intakeMotor = new Talon( 0);
    
    // intakeMotor.configFactoryDefault();

    intakeSolenoid = new Solenoid(PneumaticsModuleType.REVPH, 4);
    // intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_21_FeedbackIntegrated, 1000);
    // intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
    // intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
    // intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
    // intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255);
    // intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
    // intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255);
    // intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
    // intakeMotor.setControlFramePeriod(ControlFrame.Control_3_General, 25);
    // intakeMotor.setControlFramePeriod(ControlFrame.Control_6_MotProfAddTrajPoint, 500);
    // intakeMotor.setControlFramePeriod(ControlFrame.Control_4_Advanced, 25);


  }

  // public void configMotors(){
  //   intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_21_FeedbackIntegrated, 1000);
  //   intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
  //   intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
  //   intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
  //   intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255);
  //   intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
  //   intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255);
  //   intakeMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
  //   intakeMotor.setControlFramePeriod(ControlFrame.Control_3_General, 25);
  //   intakeMotor.setControlFramePeriod(ControlFrame.Control_6_MotProfAddTrajPoint, 500);
  //   intakeMotor.setControlFramePeriod(ControlFrame.Control_4_Advanced, 25);

  // }
/** Runs the intake motors off percent output
 *  @param power double for the speed of the motor
 * */ 
  public void runIntake(double power){
    intakeMotor.set( power);
  }

  /** Deploys the intake pneumatics
    @param state boolean for deployed or stowed
    */
  public void deployIntake(boolean state){
    intakeSolenoid.set(state);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
