// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  NetworkTableEntry camTran = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran");
  
  public static double[] camTranArray = {0.0,0.0,0.0,0.0,0.0,0.0};
  public static void setLights(int lightMode){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(lightMode);

  }

  public void updateCamTran(){
    camTranArray = camTran.getDoubleArray(camTranArray);
  }

 


  public static double distanceFromTarget(){
    return camTranArray[2];
  }

  

  public double tx() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  }

  public static double ty() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  }

  public double tyradian() {
    return Math.toRadians(ty());
  }

  public double tvert() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tvert").getDouble(0);
  }

  public double thor() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("thor").getDouble(0);
  }

  public double tlong() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tlong").getDouble(0);
  }

  public double tshort() {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tshort").getDouble(0);
  }

  public double pixels() {
    return thor() * tvert();
  }

  public void pipeline(int x) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setDouble(x);
  }

  public static double getPipeline() {

    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").getDouble(0);
  }

  public static double hasTarget(){
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
 
  }

  public boolean crosshairOnTarget(){
    if(hasTarget()==1){
      //need to adjust this threshold. maybe can change depending on distance from target
      return tx()<1;
    }
    else{
      return false;
    }
  }
  /** Creates a new Limelight. */
  // public Limelight() {}

  @Override
  public void periodic() {
    updateCamTran();

    // This method will be called once per scheduler run
  }
}
