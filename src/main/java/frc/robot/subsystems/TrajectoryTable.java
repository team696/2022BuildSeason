// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TrajectoryTable extends SubsystemBase {
  /** Creates a new TrajectoryTable. */
  public TrajectoryTable() {}

  /* IDK IF THIS WILL WORK IM BAD WITH ARRAYS */
  /**
   * An array for what hood angles correspond to each distance in feet.
   */
   public  double distanceToHoodAngle[] = {
     0.122,  /* 0 */
     0.184,  /* 1 */
     0.246,  /* 2 */
     0.308,  /* 3 */
     0.37,  /* 4 */
     0.432,  /* 5 */
     0.460,  /* 6 */
     0.556,  /* 7 */
     0.618,  /* 8 */
     0.66,  /* 9 */
     0.72,  /* 10 */
     0.742,  /* 11 */
     0.82,  /* 12 */
     0.866,  /* 13 */
     0.945,  /* 14 */
     1.023,  /* 15 */
     1.052,  /* 16 */
     1.114, /* 17 */
     1.176, /* 18 */
     1.238, /* 19 */
     1.2,  /* 20 */ 
     1.2,
     1.2,
     1.2,
     1.2,
     1.2,
     1.2,
     1.2,
     1.2,
     1.2,
     1.2,
     1.2,
     1.2,
     1.2


   };


  //  50,  /* 0 */   1
  //  53,  /* 1 */    4
  //  56,  /* 2 */    7
  //  59,  /* 3 */    10
  //  50,  /* 4 */    
  //  50,  /* 5 */    1
  //  57,  /* 6 */    8
  //  61,  /* 7 */    12
  //  67,  /* 8 */    18
  //  72,  /* 9 */    23
  //  79,  /* 10 */   30
  //  81,  /* 11 */   32
  //  86,  /* 12 */   37
  //  88,  /* 13 */   39
  //  92,  /* 14 */   43
  //  94,  /* 15 */   45
  //  96,  /* 16 */   47
  //  99, /* 17 */    50
  //  102, /* 18 */   53
  //  107, /* 19 */   58
  //  110  /* 20 */   61

   
/**
 * An array for what shooter speeds corresponds to each distance in feet.
 */
   public double distanceToShooterSpeed[] = {
     2000,    /* 0 */
     2100,    /* 1 */
     2200,    /* 2 */
     2300,    /* 3 */
     2200,    /* 4 */
     2200,    /* 5 */
     2300,    /* 6 */
     2400,    /* 7 */
     2500,    /* 8 */
     2600,    /* 9 */
     2700,    /* 10 */
     2750,    /* 11 */
     2800,    /* 12 */
     2850,    /* 13 */
     2900,    /* 14 */
     3100,    /* 15 */
     3100,    /* 16 */
     3200,    /* 17 */
     3250,    /* 18 */
     3700,    /* 19 */
     4000,     /* 20 */
     4000,
     4000,
     4000,
     4000,
     4000,
     4000,
     4000,
     4000,
     4000,
     4000,
     4000,
     4000,
   };

  
}
