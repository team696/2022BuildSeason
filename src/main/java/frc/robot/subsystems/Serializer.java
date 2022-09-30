// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Serializer extends SubsystemBase {
 public TalonFX leftSerMotor;
 int counter = 0;
 ;
 
public Talon rightSerMotor;
  public DigitalInput beamBreak = new DigitalInput(6);
  public final  I2C.Port i2cPort = I2C.Port.kMXP;
  public final ColorSensorV3 colorSensor  = new ColorSensorV3(i2cPort);
  public Color detectedColor;
  public SendableChooser<String> allianceColor;
  public String chosenAlliance;

  public String blueAlliance;
  public String redAlliance;
  public String practiceMode;
  Limelight limelight;
  TrajectoryTable trajectoryTable;

  /** Creates a new Serializer. */
  public Serializer() {
    // i2cPort = I2C.Port.kOnboard;
    // Color detectedColor = colorSensor.getColor();
    // SmartDashboard.putNumber("RED ", detectedColor.red);
    detectedColor = colorSensor.getColor();
    

    limelight = new Limelight();
    trajectoryTable = new TrajectoryTable();
    leftSerMotor = new TalonFX(Constants.Serializer.leftSerMotorport, "Alex");
    rightSerMotor = new Talon(Constants.Serializer.rightSerMotor);

    allianceColor = new SendableChooser<>();


    redAlliance = "Red";
    blueAlliance = "Blue";
    practiceMode = "Practice";
    
    
    leftSerMotor.configFactoryDefault();
    leftSerMotor.setSensorPhase(Constants.Serializer.serializerMotorSensorPhase);
    leftSerMotor.setInverted(Constants.Serializer.serializerMotorInverted);
    leftSerMotor.setNeutralMode(NeutralMode.Brake);
    leftSerMotor.configPeakOutputForward(Constants.Serializer.peakOutput);
    leftSerMotor.configPeakOutputReverse(-Constants.Serializer.peakOutput);
    leftSerMotor.configForwardSoftLimitEnable(false);
    leftSerMotor.configReverseSoftLimitEnable(false);
    leftSerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
    leftSerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
    leftSerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
    leftSerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255);
    leftSerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
    leftSerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255);
    leftSerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
    // leftSerMotor.setControlFramePeriod(ControlFrame.Control_3_General, 25);
    // leftSerMotor.setControlFramePeriod(ControlFrame.Control_6_MotProfAddTrajPoint, 500);
    // leftSerMotor.setControlFramePeriod(ControlFrame.Control_4_Advanced, 25);


    

    // rightSerMotor.configFactoryDefault();
    // rightSerMotor.setSensorPhase(Constants.Serializer.serializerMotorSensorPhase);
    // rightSerMotor.setInverted(Constants.Serializer.serializerMotorInverted);
    // rightSerMotor.setNeutralMode(NeutralMode.Brake);
    // rightSerMotor.configPeakOutputForward(Constants.Serializer.peakOutput);
    // rightSerMotor.configPeakOutputReverse(-Constants.Serializer.peakOutput);
    // rightSerMotor.configForwardSoftLimitEnable(false);
    // rightSerMotor.configReverseSoftLimitEnable(false);
    // rightSerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
    // rightSerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
    // rightSerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
    // rightSerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255);
    // rightSerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
    // rightSerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255);
    // rightSerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
    // rightSerMotor.setControlFramePeriod(ControlFrame.Control_3_General, 25);
    // rightSerMotor.setControlFramePeriod(ControlFrame.Control_6_MotProfAddTrajPoint, 500);
    // rightSerMotor.setControlFramePeriod(ControlFrame.Control_4_Advanced, 25);


  }
public void getColorSensor(){
   detectedColor = colorSensor.getColor();
}
  public double getRed(){
    return detectedColor.red;
  }

  public double getBlue(){
    return detectedColor.blue;
  }

  public double getGreen(){
    return detectedColor.green;
  }

  public void getAllianceColor(){
chosenAlliance = allianceColor.getSelected();
  }


  public boolean isBallWrong(){
    getColorSensor();
    if(chosenAlliance == "Red"){
      // java.awt.Color.RGBtoHSB( (int) getRed()*255, (int) getGreen()*255, (int) getBlue()*255,);
      if(getBlue() > 0.28 && getBlue() < 0.45 &&
      getRed() > 0.0 && getRed() < 0.28 &&
      getGreen() > 0.37 && getGreen()  < 0.49){
        return true;
      }
      else{
        return false;
      }
    }

     if(chosenAlliance == "Blue"){
      if(getBlue() > 0.00 && getBlue() < 0.28 &&
      getRed() > 0.35 && getRed() < 0.6 &&
      getGreen() > 0.32 && getGreen() < 0.45){
         return true;
       }
   else{
     return false;
   }
    }

    if (chosenAlliance == "Practice"){
      return false;
    }
    else{
      return false;
    }

    
  }



  


  public void configMotors(){
    leftSerMotor.configFactoryDefault();
    leftSerMotor.setSensorPhase(Constants.Serializer.serializerMotorSensorPhase);
    leftSerMotor.setInverted(Constants.Serializer.serializerMotorInverted);
    leftSerMotor.setNeutralMode(NeutralMode.Brake);
    leftSerMotor.configPeakOutputForward(Constants.Serializer.peakOutput);
    leftSerMotor.configPeakOutputReverse(-Constants.Serializer.peakOutput);
    leftSerMotor.configForwardSoftLimitEnable(false);
    leftSerMotor.configReverseSoftLimitEnable(false);
    // leftSerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
    // leftSerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
    // leftSerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
    // leftSerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255);
    // leftSerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
    // leftSerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255);
    // leftSerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
    // leftSerMotor.setControlFramePeriod(ControlFrame.Control_3_General, 25);
    // leftSerMotor.setControlFramePeriod(ControlFrame.Control_6_MotProfAddTrajPoint, 500);
    // leftSerMotor.setControlFramePeriod(ControlFrame.Control_4_Advanced, 25);


    

    // rightSerMotor.configFactoryDefault();
    // rightSerMotor.setSensorPhase(Constants.Serializer.serializerMotorSensorPhase);
    // rightSerMotor.setInverted(Constants.Serializer.serializerMotorInverted);
    // rightSerMotor.setNeutralMode(NeutralMode.Brake);
    // rightSerMotor.configPeakOutputForward(Constants.Serializer.peakOutput);
    // rightSerMotor.configPeakOutputReverse(-Constants.Serializer.peakOutput);
    // rightSerMotor.configForwardSoftLimitEnable(false);
    // rightSerMotor.configReverseSoftLimitEnable(false);
    // rightSerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
    // rightSerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
    // rightSerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
    // rightSerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, 255);
    // rightSerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
    // rightSerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, 255);
    // rightSerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
    // rightSerMotor.setControlFramePeriod(ControlFrame.Control_3_General, 25);
    // rightSerMotor.setControlFramePeriod(ControlFrame.Control_6_MotProfAddTrajPoint, 500);
    // rightSerMotor.setControlFramePeriod(ControlFrame.Control_4_Advanced, 25);
  }
  /**
   * Method for running the serializer motors.
   * @param percent percent output for the top motor.
   * @param percent2 percent output for the bottom  motor.
   */
    public void runSerMotors(double percent, double percent2){
      leftSerMotor.set(TalonFXControlMode.PercentOutput, percent);
      rightSerMotor.set( percent2);
    }

    
  public double getRequiredShootSpeed(){
    int distance;
    double limelightDistance;
    double speed;
    // double speed;
    limelightDistance = limelight.getDistance()/12;
    distance = (int)Math.round(limelightDistance);

    if(distance <21){
    speed  = trajectoryTable.distanceToShooterSpeed[distance];
    // speed = trajectoryTable.distanceToShooterSpeed[distance];
    // limelight.setLights(mode);
    // shooterHood.moveActuators(angle);
    return speed + 100;
    }
    else{
      return 3100;
    }
    
  }

  
  

  @Override
  public void periodic() {

    // Color detectedColor = colorSensor.getColor();
    // getColorSensor();
    getAllianceColor();

    allianceColor.setDefaultOption("Red Alliance", redAlliance);
    allianceColor.addOption("Blue Alliance", blueAlliance);
    allianceColor.addOption("PRACTICE MODE", practiceMode);

    SmartDashboard.putData(allianceColor);
    // SmartDashboard.putNumber("GREEN ", getGreen());
    // SmartDashboard.putNumber("RED ", getRed());
    // SmartDashboard.putNumber("BLUE ", getBlue());
    // SmartDashboard.putNumber("RED ", get.red);
    // double green = getGreen();
    // SmartDashboard.putNumber("GREEN", green);

  }
}


