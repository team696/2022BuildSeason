// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CameraServerJNI;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoCamera;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.ClimbCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DIOSub;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Serializer;
import frc.robot.subsystems.Shooter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static CTREConfigs ctreConfigs;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  public static  CameraServer camServ;
  public VideoCamera cam;
  // public AddressableLED rightLED;
  // public AddressableLEDBuffer rightBuffer;

  // public AddressableLED leftLED;
  // public AddressableLEDBuffer leftBuffer;


  // public final  I2C.Port i2cPort = I2C.Port.kOnboard;
  // public final ColorSensorV3 colorSensor  = new ColorSensorV3(i2cPort);
  // private Serializer serializer;
  // private Climber climber;
  // // private Intake intake;
  // private Shooter shooter;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // m_robotContainer.ledSub.setRightLEDs(100, 100, 100);
    // m_robotContainer.ledSub.armenianFlag();

    // serializer = new Serializer();
    // climber = new Climber();
    // // intake = new Intake();
    // shooter = new Shooter();
    ctreConfigs = new CTREConfigs();
    // shooter.configMotors();
    // climber.configMotors();
    // intake.configMotors();
    // serializer.configMotors();\

    // cam = new vide
        // cam.setVideoMode(PixelFormat.kYUYV, 320, 240, 30);
        

    CameraServer.startAutomaticCapture().setVideoMode(PixelFormat.kMJPEG, 176, 144, 30);
    
//  PWM port 9
// //     Must be a PWM header, not MXP or DIO
    // rightLED = new AddressableLED(9);
    // // leftLED = new AddressableLED(8);
    // // leftLED = new AddressableLED(9);

    // // Reuse buffer
    // // Default to a length of 60, start empty output
    // // Length is expensive to set, so only set it once, then just update data
    // rightBuffer = new AddressableLEDBuffer(60);
    // rightLED.setLength(rightBuffer.getLength());

    // // Set the data
    // rightLED.setData(rightBuffer);
    // rightLED.start();




    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    // leftBuffer = new AddressableLEDBuffer(60);
    // leftLED.setLength(leftBuffer.getLength());

    // // Set the data
    // leftLED.setData(rightBuffer);
    // leftLED.start();



    
    


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
    // m_robotContainer.ledSub.setLeftLEDs(0, 0, 255);
    // m_robotContainer.ledSub.setRightLEDs(0, 0, 255);

    // Color detectedColor = colorSensor./getColor();
// for (var i = 0; i < rightBuffer.getLength(); i++) {
//    // Sets the specified LED to the RGB values for red
//    rightBuffer.setRGB(i, 0, 255, 0);
// }

// rightLED.setData(rightBuffer);

// for (var i = 0; i < leftBuffer.getLength(); i++) {
//   // Sets the specified LED to the RGB values for red
//   leftBuffer.setRGB(i, 0, 255, 0);
// }

// leftLED.setData(rightBuffer);
    // SmartDashboard.putNumber("RED ", detectedColor.red);
    SmartDashboard.putBoolean("DIO 0 (DH L B)", DIOSub.DH_L_B);
    SmartDashboard.putBoolean("DIO 1 (DH L T)", DIOSub.DH_L_T);
    SmartDashboard.putBoolean("DIO 2 (DH R B)", DIOSub.DH_R_B);
    SmartDashboard.putBoolean("DIO 5 (DH R T)", DIOSub.DH_R_T);
    SmartDashboard.putBoolean("DIO 4 (SH L)", DIOSub.SH_L);
    SmartDashboard.putBoolean("DIO 3 (SH R)", DIOSub.SH_R);
    SmartDashboard.putNumber("Shooter Speed", m_robotContainer.shooter.getShooterRPM());
    SmartDashboard.putNumber("HOOD ANGLE ", m_robotContainer.shooterHood.getEncoderPos());
    SmartDashboard.putBoolean("DIO 6 BEAM BREAK ", m_robotContainer.serializer.beamBreak.get());
    // SmartDashboard.putNumber("Pressure ", m_robotContainer.pneumatics.getPressure());
    SmartDashboard.putNumber("DISTANCE TO TARGET", m_robotContainer.limelight.getDistance()/12);
    SmartDashboard.putNumber("LL TX", m_robotContainer.limelight.tx());
    SmartDashboard.putBoolean("LOCKED ", m_robotContainer.limelight.crosshairOnTarget());
    // SmartDashboard.putNumber("GYRO ", m_robotContainer.s_Swerve.gyro.getYaw());

    // SmartDashboard.putNumber("RED", m_robotContainer.serializer.getRed());
    // SmartDashboard.putNumber("GREEN", m_robotContainer.serializer.getGreen());
    // SmartDashboard.putNumber("BLUE ", m_robotContainer.serializer.getBlue());
    
    

    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    // m_robotContainer.ledSub.setRightLEDs(0, 30, 0);
  //  m_robotContainer.ledSub.armenianFlag();
  //  m_robotContainer.ledSub.americanFlag();
  //  m_robotContainer.ledSub.italianFlag();
  // m_robotContainer.ledSub.japaneseFlag();
  }

  @Override
  public void disabledPeriodic() {
    // m_robotContainer.ledSub.pinkGreen_Alt();
    // m_robotContainer.ledSub.pinkGreen_Brth();;
    //  m_robotContainer.ledSub.armenianFlag();
    //  m_robotContainer.ledSub.americanFlag();
    //  m_robotContainer.ledSub.italianFlag();
    //  m_robotContainer.ledSub.japaneseFlag();
    // m_robotContainer.ledSub.pinkGreen_Alt();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // m_robotContainer.ledSub.setRightLEDs(100, 100, 100);
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
    // m_robotContainer.ledSub.setRightLEDs(10, 10, 10);
   
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
