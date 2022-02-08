// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {

private WheelDrive backRight;
private WheelDrive backLeft;
private WheelDrive frontRight;
private WheelDrive frontLeft;
  /** Creates a new SwerveDrive. */
  public SwerveDrive(WheelDrive backRight, WheelDrive backLeft, WheelDrive frontRight, WheelDrive frontLeft) {
    this.backRight = backRight;
    this.backLeft = backLeft;
    this.frontRight = frontRight;
    this.frontLeft = frontLeft;

  }

public final double L = 27;
public final double W = 24;

public void drive (double x1, double y1, double x2) {
  double r = Math.sqrt((L * L) + (W * W));
  y1 = y1 *= -1;

  double a = x1 - x2 * (L / r);
  double b = x1 + x2 * (L / r);
  double c = y1 - x2 * (W / r);
  double d = y1 + x2 * (W / r);

  double backRightSpeed = Math.sqrt ((a * a) + (d * d));
  double backLeftSpeed = Math.sqrt ((a * a) + (c * c));
  double frontRightSpeed = Math.sqrt ((b * b) + (d * d));
  double frontLeftSpeed = Math.sqrt ((b * b) + (c * c));

  double backRightAngle = Math.atan2 (a, d) / Math.PI;
  double backLeftAngle = Math.atan2 (a, c) / Math.PI;
  double frontRightAngle = Math.atan2 (b, d) / Math.PI ;
  double frontLeftAngle = Math.atan2 (b, c) / Math.PI;

  backRight.drive (backRightSpeed, backRightAngle);
  backLeft.drive (backLeftSpeed, backLeftAngle);
  frontRight.drive (frontRightSpeed, frontRightAngle);
  frontLeft.drive (frontLeftSpeed, frontLeftAngle);

  // backRight.drive (x1, x2);
  // backLeft.drive (x1, x2);
  //  frontRight.drive (x1, x2);
  // frontLeft.drive (x1, x2);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
