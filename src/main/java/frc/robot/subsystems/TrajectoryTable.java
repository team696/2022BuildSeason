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
     50,  /* 0 */
     53,  /* 1 */
     56,  /* 2 */
     59,  /* 3 */
     50,  /* 4 */
     50,  /* 5 */
     57,  /* 6 */
     61,  /* 7 */
     67,  /* 8 */
     72,  /* 9 */
     79,  /* 10 */
     81,  /* 11 */
     86,  /* 12 */
     88,  /* 13 */
     92,  /* 14 */
     94,  /* 15 */
     96,  /* 16 */
     99, /* 17 */
     102, /* 18 */
     107, /* 19 */
     110  /* 20 */
   };

   public double distanceToShooterSpeed[] = {
     2000,    /* 0 */
     2100,    /* 1 */
     2200,    /* 2 */
     2300,    /* 3 */
     2200,    /* 4 */
     2200,    /* 5 */
     2300,    /* 6 */
     2400,    /* 7 */
     2450,    /* 8 */
     2600,    /* 9 */
     2700,    /* 10 */
     2700,    /* 11 */
     2800,    /* 12 */
     2800,    /* 13 */
     2900,    /* 14 */
     3000,    /* 15 */
     3100,    /* 16 */
     3200,    /* 17 */
     3200,    /* 18 */
     3700,    /* 19 */
     4000     /* 20 */
   };

  
}
