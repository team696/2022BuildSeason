// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DIOSub;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    // climber = new Climber();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putBoolean("DIO 0 (DH L B)", DIOSub.DH_L_B);
    SmartDashboard.putBoolean("DIO 1 (DH L T)", DIOSub.DH_L_T);
    SmartDashboard.putBoolean("DIO 2 (DH R B)", DIOSub.DH_R_B);
    SmartDashboard.putBoolean("DIO 5 (DH R T)", DIOSub.DH_R_T);
    SmartDashboard.putBoolean("DIO 4 (SH L)", DIOSub.SH_L);
    SmartDashboard.putBoolean("DIO 3 (SH R)", DIOSub.SH_R);
    SmartDashboard.putNumber("Climber Stick ", RobotContainer.controlPanel.getRawAxis(0));
    SmartDashboard.putNumber("Shooter Speed", m_robotContainer.shooter.getShooterRPM());
    SmartDashboard.putNumber("Hood Axis", Math.round(m_robotContainer.controlPanel.getRawAxis(1)*128));
    SmartDashboard.putNumber("HOOD ANGLE ", m_robotContainer.shooterHood.servoPosition());
    SmartDashboard.putBoolean("DIO 6 BEAM BREAK ", m_robotContainer.serializer.beamBreak.get());
    SmartDashboard.putNumber("Climber Angle", m_robotContainer.climber.getClimberPos() / 130.666); 

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // m_robotContainer.shooterHood.rightActuator.setAngle(0);
    // m_robotContainer.shooterHood.leftActuator.setAngle(0);
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
    // new ClimbCommand(climber, m_robotContainer.controlPanel.getRawAxis(0), true , true);
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
