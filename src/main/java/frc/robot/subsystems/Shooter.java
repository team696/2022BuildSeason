// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
public   WPI_TalonFX leftShooterMotor;
 public WPI_TalonFX rightShooterMotor;
 public double shooterSpeed;
  Limelight limelight;
  TrajectoryTable trajectoryTable;
  double lldistance;

  /** Creates a new Shooter. */
  public Shooter() {
    limelight = new Limelight();
    trajectoryTable = new TrajectoryTable();

    leftShooterMotor = new WPI_TalonFX(41, "Alex");
    rightShooterMotor = new WPI_TalonFX(40, "Alex");

    leftShooterMotor.configFactoryDefault();
    leftShooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    leftShooterMotor.setSensorPhase(true );
     leftShooterMotor.setNeutralMode(NeutralMode.Coast);
    leftShooterMotor.configPeakOutputForward(1);
    leftShooterMotor.configPeakOutputReverse(-1);
    leftShooterMotor.configNominalOutputForward(0);
    leftShooterMotor.configNominalOutputReverse(0);
    leftShooterMotor.configClosedloopRamp(0.25);
    leftShooterMotor.config_kP(0, 0.1);
    leftShooterMotor.config_kI(0, 0.0);
    leftShooterMotor.config_kD(0, 0.0);
    leftShooterMotor.config_kF(0, 0.06);
    leftShooterMotor.configAllowableClosedloopError(0, 10);
    leftShooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
    leftShooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
    leftShooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
    leftShooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255);
    leftShooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
    leftShooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255);
    leftShooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
    // leftShooterMotor.setControlFramePeriod(ControlFrame.Control_3_General, 25);
    // leftShooterMotor.setControlFramePeriod(ControlFrame.Control_6_MotProfAddTrajPoint, 500);
    // leftShooterMotor.setControlFramePeriod(ControlFrame.Control_4_Advanced, 25);

    rightShooterMotor.configFactoryDefault();
    rightShooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 1, 10);
    rightShooterMotor.setSensorPhase(false );
    rightShooterMotor.setInverted(true);
    rightShooterMotor.setNeutralMode(NeutralMode.Coast);
    rightShooterMotor.configPeakOutputForward(1 );
    rightShooterMotor.configPeakOutputReverse(-1);
    rightShooterMotor.configNominalOutputForward(0);
    rightShooterMotor.configNominalOutputReverse(0);
    rightShooterMotor.configClosedloopRamp(0.25);
    rightShooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
    rightShooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
    rightShooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
    rightShooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255);
    rightShooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
    rightShooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255);
    rightShooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
    // rightShooterMotor.setControlFramePeriod(ControlFrame.Control_3_General, 25);
    // rightShooterMotor.setControlFramePeriod(ControlFrame.Control_6_MotProfAddTrajPoint, 500);
    // rightShooterMotor.setControlFramePeriod(ControlFrame.Control_4_Advanced, 25);

    rightShooterMotor.follow(leftShooterMotor);
   
  }

  public void configMotors(){
    leftShooterMotor.configFactoryDefault();
    leftShooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    leftShooterMotor.setSensorPhase(true );
     leftShooterMotor.setNeutralMode(NeutralMode.Coast);
    leftShooterMotor.configPeakOutputForward(1);
    leftShooterMotor.configPeakOutputReverse(-1);
    leftShooterMotor.configNominalOutputForward(0);
    leftShooterMotor.configNominalOutputReverse(0);
    leftShooterMotor.configClosedloopRamp(0.25);
    leftShooterMotor.config_kP(0, 0.1);
    leftShooterMotor.config_kI(0, 0.0);
    leftShooterMotor.config_kD(0, 0.0);
    leftShooterMotor.config_kF(0, 0.06);
    leftShooterMotor.configAllowableClosedloopError(0, 10);
    leftShooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
    leftShooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
    leftShooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
    leftShooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255);
    leftShooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
    leftShooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255);
    leftShooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
    leftShooterMotor.setControlFramePeriod(ControlFrame.Control_3_General, 25);
    leftShooterMotor.setControlFramePeriod(ControlFrame.Control_6_MotProfAddTrajPoint, 500);
    leftShooterMotor.setControlFramePeriod(ControlFrame.Control_4_Advanced, 25);

    rightShooterMotor.configFactoryDefault();
    rightShooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 1, 10);
    rightShooterMotor.setSensorPhase(false );
    rightShooterMotor.setInverted(true);
    rightShooterMotor.setNeutralMode(NeutralMode.Coast);
    rightShooterMotor.configPeakOutputForward(1 );
    rightShooterMotor.configPeakOutputReverse(-1);
    rightShooterMotor.configNominalOutputForward(0);
    rightShooterMotor.configNominalOutputReverse(0);
    rightShooterMotor.configClosedloopRamp(0.25);
    rightShooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
    rightShooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
    rightShooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
    rightShooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255);
    rightShooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
    rightShooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255);
    rightShooterMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
    rightShooterMotor.setControlFramePeriod(ControlFrame.Control_3_General, 25);
    rightShooterMotor.setControlFramePeriod(ControlFrame.Control_6_MotProfAddTrajPoint, 500);
    rightShooterMotor.setControlFramePeriod(ControlFrame.Control_4_Advanced, 25);

    rightShooterMotor.follow(leftShooterMotor);

  }

  public void getLimelightDistance(){
    lldistance = limelight.getDistance();
  }  

  /**
   * Method used to convert RPM  to TalonFX sensor units.
   * (Inverse of talonFXtoRPM)
   * @param rpm  The RPM you want to convert.
   * @return the TalonFX sensor units.
   */
  public double rpmToTalonFX(double rpm){
    double retval = rpm / 0.667 / 60 / 10 * 2048;
    return retval;
  }

  /**
   * Method used to convert TalonFX units into RPM.
   * (Inverse of rpmToTalonFX)
   * @param speed TalonFX sensor units you want to convert \.
   * @return The RPM equivalent of the TalonFX sensor units.
   */
  public double talonFXtoRPM(double speed){     // talonFX = encoder ticks per 100ms
    double rpm = speed/2048 * 10 * 60 * 0.667;  //  (rotations per 1/10s) * encoder updates per second * rps to rpm * gear ratio
    return rpm;
  }

/**
 * 
 * @return the RPM of the shooter in TalonFX sensor units.
 */
  public double getShooterRPM(){
    return talonFXtoRPM(leftShooterMotor.getSelectedSensorVelocity());

  }

  /**
   * Method for controlling the shooter wheels using percent output.
   * @param percent output of the motor from -1 to 1.
   */
  public void setShooterPercent(double percent){
    leftShooterMotor.set(TalonFXControlMode.PercentOutput, percent);
  }

/**
 * Uses the limelight and the trajectory table 
 * @return the required shooter wheel speed using the distance from the target.
 */

 public void getShootSpeed(){
   int distance;
   double limelightDistance;
   double speed;
   double last_speed = 3000;
   // double speed;
   limelightDistance = lldistance/12;
   distance = (int)Math.round(limelightDistance);

   if(distance <21){
   speed  = trajectoryTable.distanceToShooterSpeed[distance];
   // speed = (Math.sqrt(limelightDistance) * 848.972) + 200;
   last_speed = speed;

   //THIS EQUATION IS OWNING. BY CODING KING
   // 1833.96x^3+-2524.94x^2+1848.64x+1820.18

   // speed = trajectoryTable.distanceToShooterSpeed[distance];
   // limelight.setLights(mode);
   // shooterHood.moveActuators(angle);
   shooterSpeed =  speed - 50 ;
   }
   else{
     shooterSpeed =  last_speed ;
   }

 }
  public double getRequiredShootSpeed(){
    int distance; 
    double limelightDistance;
    double speed;
    double last_speed = 3000;
    // double speed;
    limelightDistance = lldistance/12;
    distance = (int)Math.round(limelightDistance); 

    if(distance <21){
    speed = Math.sqrt(Constants.Shooter.gravity * (Constants.Shooter.height + Math.sqrt(Constants.Shooter.height * Constants.Shooter.height + limelightDistance * limelightDistance )) ) * Constants.Shooter.conversionFactor;
    //speed  = trajectoryTable.distanceToShooterSpeed[distance];
    // speed = (Math.sqrt(limelightDistance) * 848.972) + 200;
    last_speed = speed;
    // speed = trajectoryTable.distanceToShooterSpeed[distance];
    // limelight.setLights(mode);
    // shooterHood.moveActuators(angle);
    return speed;
    }
    else{
      return last_speed;
    }
    
  }

  


/**
 * Method for controlling the shooter wheels using velocity control.
 * @param speed The desired speed of the shooter wheel in RPM.
 */
  public void setShooter(double speed){
    leftShooterMotor.set(TalonFXControlMode.Velocity, speed);
    // leftShooterMotor.set(1);
  }

  /**
   * Disables the shooter.
   */
  public void shooterOff(){
    leftShooterMotor.disable();
  }

  @Override
  public void periodic() {
    getLimelightDistance();
    getShootSpeed();
    // This method will be called once per scheduler run
  }
}
