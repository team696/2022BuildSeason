// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DIOSub extends SubsystemBase {
  DigitalInput DH_L_B = new DigitalInput(Constants.DOUBLEHAND_L_BOTTOM);
  DigitalInput DH_L_T = new DigitalInput(Constants.DOUBLEHAND_L_TOP);
  DigitalInput DH_R_B = new DigitalInput(Constants.DOUBLEHAND_R_BOTTOM);
  DigitalInput DH_R_T = new DigitalInput(Constants.DOUBLEHAND_R_TOP);
  DigitalInput SH_L = new DigitalInput(Constants.SINGLEHAND_L);
  DigitalInput SH_R = new DigitalInput(Constants.SINGLEHAND_R);
  /** Creates a new DIO. */
  public DIOSub() {}
  public boolean[] getSensorStates(){

    boolean out_array[]=new boolean[6];
    
    out_array[0] = DH_L_B.get();
    out_array[1] = DH_L_T.get();
    out_array[2] = DH_R_B.get();
    out_array[3] = DH_R_T.get();
    out_array[4] = SH_L.get();
    out_array[5] = SH_R.get();

    return out_array;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
