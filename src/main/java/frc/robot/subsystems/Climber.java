// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.fasterxml.jackson.annotation.JacksonInject.Value;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  //  private RobotContainer robotContainer;
  public WPI_TalonFX lClimberMotor;
  public WPI_TalonFX rClimberMotor;
  final int kUnitsPerRevolution = 2048;
  // DigitalInput DH_L_B = new DigitalInput(Constants.DOUBLEHAND_L_BOTOTM);
  // DigitalInput DH_L_T = new DigitalInput(Constants.DOUBLEHAND_L_TOP);
  // DigitalInput DH_R_B = new DigitalInput(Constants.DOUBLEHAND_R_BOTTOM);
  // DigitalInput DH_R_T = new DigitalInput(Constants.DOUBLEHAND_R_TOP);
  // DigitalInput SH_L = new DigitalInput(Constants.SINGLEHAND_L);
  // DigitalInput SH_R = new DigitalInput(Constants.SINGLEHAD_R);

  

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
    lClimberMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 1, 30);
    lClimberMotor.setNeutralMode(NeutralMode.Coast );
    // lClimberMotor.setSensorPhase(true);
    // lClimberMotor.setInverted(true);
    // lClimberMotor.config_kF(1, 1.0, 30);
    // lClimberMotor.config_kP(1, 1.0, 30);
    // lClimberMotor.config_kI(1, 0, 30);
    // lClimberMotor.config_kD(1, 0, 30);

   

  }

  public  double getClimberVoltage(){
    return lClimberMotor.getSupplyCurrent();
  }


  /**
	 * Get the current climber's position in degrees from zero, which should be set to 0 when the robot is enabled.
	 *
	 * @return Position of climber (in degrees).
   * @apiNote Uses the left climber Talon FX encoder to retrieve position, which is set 0 during teleopPeriodic().
	 */
  public double getClimberPos(){
    // return lClimberMotor.getSelectedSensorPosition() / 2048.0;
    return lClimberMotor.getSelectedSensorPosition() / 2048.0 * 360;
  }
  
  // public double climberPos(){
  //   // return lClimberMotor.getSelectedSensorPosition()/
  // }

  public void moveClimber(double percent){
    // System.out.println("Left: " + percent);
    // System.out.println("Right: " + (-percent));
    lClimberMotor.set(TalonFXControlMode.PercentOutput, percent);
    rClimberMotor.set(TalonFXControlMode.PercentOutput, -percent);

  }
  public void initialize(){
    lClimberMotor.setSelectedSensorPosition(0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
