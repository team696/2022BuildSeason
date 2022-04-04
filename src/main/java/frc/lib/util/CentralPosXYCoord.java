// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import javax.sound.sampled.LineEvent;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Limelight;

public class CentralPosXYCoord extends SubsystemBase {
  /** Creates a new CentralPosXYCoord. */
  Limelight limelight;
  AHRS navX;
  double x;
  double y;

  public CentralPosXYCoord() {
    navX = new AHRS();

    x = limelight.getDistance() * Math.sin(navX.getYaw());
    y = limelight.getDistance() * Math.cos(navX.getYaw());

    System.out.println( getX(60, 6) + "," + getY(60, 6)  );


  }

  public double getX(double dist, double angle){
    return dist * Math.sin(angle);
  }

  public double getY(double dist, double angle){
    return dist * Math.cos(angle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
