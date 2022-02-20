// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoClimbSequence;
import frc.robot.commands.AutoClimbStep1;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.DIOTest;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.commands.PneumaticsCommand1;
import frc.robot.commands.PneumaticsCommand2;
import frc.robot.commands.SerializerCommand;
import frc.robot.commands.SingleLatchRelease;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DIOSub;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Serializer;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final Serializer serializer = new Serializer();
  private final Climber climber = new Climber();
  private final Pneumatics pneumatics = new Pneumatics();
  public final DIOSub dioSub = new DIOSub();
  private final XboxController m_controller = new XboxController(0);
  public  static final Joystick controlPanel = new Joystick(2);
  private final JoystickButton leftStickButton = new JoystickButton(m_controller,  Constants.GYRO_RECALIBRATE_BUTTON) ;
  private final JoystickButton climbPneuButton1 = new JoystickButton(controlPanel, Constants.CLIMBER_DOUBLE_HAND_BUTTON);
  private final JoystickButton climbPneuButton2 = new JoystickButton(controlPanel, Constants.CLIMBER_SINGLE_HAND_BUTTON);
  private final JoystickButton singleRelockButton = new JoystickButton(controlPanel , Constants.CLIMBER_SINGLE_RELOCK_BUTTON);
  private final JoystickButton autoClimbButton = new JoystickButton(controlPanel, Constants.CLIMBER_AUTO_BUTTON);
  private final JoystickButton serializerForButton = new JoystickButton(controlPanel, 17);
  private final JoystickButton serializerRevButton = new JoystickButton(controlPanel, 12);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    climber.setDefaultCommand(new ClimbCommand(climber));
    // pneumatics.setDefaultCommand(new DIOTest(pneumatics, dioSub));

    
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new JoystickDriveCommand(
            m_drivetrainSubsystem,
            () -> -modifyAxis(m_controller.getRawAxis(Constants.TRANSLATE_X_AXIS)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(-m_controller.getRawAxis(Constants.TRANSALTE_Y_AXIS)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
            () -> -modifyAxis(-m_controller.getRawAxis(Constants.ROTATE_AXIS)) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    ));

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    serializerForButton.whenPressed(new SerializerCommand(serializer, 0.6, -0.6));
    serializerForButton.whenReleased(new SerializerCommand(serializer, 0, 0));

    serializerRevButton.whenPressed(new SerializerCommand(serializer, -0.6, 0.6));
    serializerRevButton.whenReleased(new SerializerCommand(serializer, 0, 0));

    leftStickButton.whenPressed(m_drivetrainSubsystem::zeroGyroscope); //Zero's the gyro to the robot's current direction

    //Actuation of the climber's pneumatic components via switches
    climbPneuButton1.whenPressed(new PneumaticsCommand1(pneumatics, Value.kForward));
    climbPneuButton1.whenReleased(new PneumaticsCommand1(pneumatics, Value.kReverse));

    // climbPneuButton2.whenPressed(new PneumaticsCommand2(climber, Value.kForward));
    // climbPneuButton2.whenReleased(new PneumaticsCommand2(climber, Value.kReverse));
    singleRelockButton.whenPressed(new PneumaticsCommand2(pneumatics, Value.kForward));
    // singleRelockButton.whenReleased(new ClimbCommand(climber));
    climbPneuButton2.whenPressed(new SingleLatchRelease(climber, pneumatics));
    climbPneuButton2.whenReleased(new ClimbCommand(climber));

    autoClimbButton.whenPressed(new AutoClimbSequence(climber, pneumatics, dioSub));
    // autoClimbButton.whenPressed(new AutoClimbStep1(climber, pneumatics, dioSub));

    // autoClimbButton.whenReleased(new ClimbCommand(climber)); 
    // climbPneuButton2.whenReleased(new PneumaticsCommand2(climber, Value.kReverse));

  }

  /**\
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new InstantCommand();
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, Constants.DEADBAND_VALUE);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }
}
