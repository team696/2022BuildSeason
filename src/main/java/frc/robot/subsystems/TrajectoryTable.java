// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TrajectoryTable extends SubsystemBase {
  /** Creates a new TrajectoryTable. */
  public TrajectoryTable() {}

  /* IDK IF THIS WILL WORK IM BAD WITH ARRAYS */
   public  double distanceToHoodAngle[] = {
     50,
     53,
     56,
     59,
     62,
     65,
     68,
     71,
     74,
     77,
     80,
     83,
     86,
     89,
     92,
     95,
     98,
     101,
     104,
     107,
     110
   };

   public double distanceToShooterSpeed[] = {
     2000,
     2100,
     2200,
     2300,
     2400,
     2500,
     2600,
     2700,
     2800,
     2900,
     3000,
     3100,
     3200,
     3300,
     3400,
     3500,
     3600,
     3700,
     3800,
     3900,
     4000
   };

  
}
