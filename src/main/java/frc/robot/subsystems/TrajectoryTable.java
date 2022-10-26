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
     0,  /* 3 */
     0.33,  /* 4 */
     0.375,  /* 5 */
     0.475,  /* 6 */
     0.566,  /* 7 */
     0.65,  /* 8 */
     0.7,  /* 9 */
     0.72,  /* 10 */
     1.05,  /* 11 */
     0.95,  /* 12 */
     0.866,  /* 13 */
     0.945,  /* 14 */
     1.1,  /* 15 */
     1.092,  /* 16 */
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
     2000,    /* 3 */
     2000,    /* 4 */
     2175,    /* 5 g */
     2300,    /* 6 g */
     2400,    /* 7 g */
     2400,    /* 8 g */
     2550,    /* 9 g */ 
     2550,    /* 10 g */
     2750,    /* 11 g */
     2750,    /* 12 g */
     2850,    /* 13 g */
     2925,    /* 14 g */
     3050,    /* 15 g */
     3200,    /* 16 g */ 
     3400,    /* 17 g   */
     3450,    /* 18 */
     3900,    /* 19 */
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
