// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterHood extends SubsystemBase {
  public Servo leftActuator;
 public  Servo rightActuator;
  /** Creates a new ShooterHood. */
  public ShooterHood() {
    leftActuator = new Servo(6);
    rightActuator = new Servo(7);

    // leftActuator.setAngle(0);
    // rightActuator.setAngle(0);
  }

  public void moveActuators(double position){
    leftActuator.setAngle(position);
    rightActuator.setAngle(position);
    System.out.println("Imhere" + position);
  }

  public double servoPosition(){
return leftActuator.getAngle();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
