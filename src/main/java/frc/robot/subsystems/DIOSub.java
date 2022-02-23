// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DIOSub extends SubsystemBase {
  DigitalInput sensor_DH_L_B = new DigitalInput(Constants.DOUBLEHAND_L_BOTTOM);
  DigitalInput sensor_DH_L_T = new DigitalInput(Constants.DOUBLEHAND_L_TOP);
  DigitalInput sensor_DH_R_B = new DigitalInput(Constants.DOUBLEHAND_R_BOTTOM);
  DigitalInput sensor_DH_R_T = new DigitalInput(Constants.DOUBLEHAND_R_TOP);
  DigitalInput sensor_SH_L = new DigitalInput(Constants.SINGLEHAND_L);
  DigitalInput sensor_SH_R = new DigitalInput(Constants.SINGLEHAND_R);

  public static boolean DH_L_B = false;
  public static boolean DH_L_T = false;
  public static boolean DH_R_B = false;
  public static boolean DH_R_T = false;
  public static boolean SH_L = false;
  public static boolean SH_R = false;
  /** Creates a new DIO. */
  public DIOSub() {}

  public boolean[] getSensorStates(){

    boolean out_array[]=new boolean[6];


    
    out_array[0] = sensor_DH_L_B.get();
    out_array[1] = sensor_DH_L_T.get();
    out_array[2] = sensor_DH_R_B.get();
    out_array[3] = sensor_DH_R_T.get();
    out_array[4] = sensor_SH_L.get();
    out_array[5] = sensor_SH_R.get();

    return out_array;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    DH_L_B = sensor_DH_L_B.get();
    DH_L_T = sensor_DH_L_B.get();
    DH_R_B = sensor_DH_L_B.get();
    DH_R_T = sensor_DH_L_B.get();
    SH_L = sensor_DH_L_B.get();
    SH_R = sensor_DH_L_B.get();
    }
}
