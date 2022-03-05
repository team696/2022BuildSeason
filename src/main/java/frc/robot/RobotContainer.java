// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.AutoClimbSequence;
import frc.robot.commands.AutoClimbSequenceNew;
import frc.robot.commands.ClimbCommand;

import frc.robot.commands.IntakeCommand;
import frc.robot.commands.DIOTest;
import frc.robot.commands.JoystickDriveCommand;
import frc.robot.commands.LimelightHoodLock;
import frc.robot.commands.PneumaticsCommand1;
import frc.robot.commands.PneumaticsCommand2;
import frc.robot.commands.SerializerCommand;
import frc.robot.commands.SerializerRevCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShooterFinished;
import frc.robot.commands.ShooterHoodCommand;
import frc.robot.commands.SingleLatchRelease;
import frc.robot.commands.SpitTopBall;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DIOSub;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Serializer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.TrajectoryTable;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  public final Serializer serializer = new Serializer();
  public final Climber climber = new Climber();
  public  final Pneumatics pneumatics = new Pneumatics();
  public final Shooter shooter = new Shooter();
  public final DIOSub dioSub = new DIOSub();
  public final ShooterHood shooterHood = new ShooterHood();
  public final Intake intake = new Intake();
  public final Limelight limelight = new Limelight();
  public final TrajectoryTable trajectoryTable = new TrajectoryTable();

  private final XboxController m_controller = new XboxController(0);
  public  static final Joystick controlPanel = new Joystick(2);

  private final JoystickButton leftStickButton = new JoystickButton(m_controller,  Constants.GYRO_RECALIBRATE_BUTTON) ;
  private final JoystickButton climbPneuButton1 = new JoystickButton(controlPanel, Constants.CLIMBER_DOUBLE_HAND_BUTTON);
  private final JoystickButton climbPneuButton2 = new JoystickButton(controlPanel, Constants.CLIMBER_SINGLE_HAND_BUTTON);
  private final JoystickButton singleRelockButton = new JoystickButton(controlPanel , Constants.CLIMBER_SINGLE_RELOCK_BUTTON);
  private final JoystickButton autoClimbButton = new JoystickButton(controlPanel, Constants.CLIMBER_AUTO_BUTTON);
  private final JoystickButton serializerForButton = new JoystickButton(controlPanel, 7);
  private final JoystickButton serializerRevButton = new JoystickButton(controlPanel, 1);
  private final JoystickButton shooterSpinup = new JoystickButton(controlPanel, 16);
  private final JoystickButton shooterHoodUp = new JoystickButton(controlPanel, 9);
  private final JoystickButton spitBallButton = new JoystickButton(controlPanel, 14);
  private final JoystickButton lockOnSwitch = new JoystickButton(m_controller, 4);
  private final JoystickButton fireButton = new JoystickButton(controlPanel, 8);

  static public boolean isShooting = false;




  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    climber.setDefaultCommand(new ClimbCommand(climber));
    shooterHood.setDefaultCommand(new ShooterHoodCommand(shooterHood));
   
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
/* ================================= SERIALIZER/INTAKE ================================= */

    serializerForButton.whenPressed(new SerializerCommand(serializer, 0.2, -0.6).alongWith(new IntakeCommand(intake, -0.5, true)));
    serializerForButton.whenReleased(new SerializerCommand(serializer, 0, 0).alongWith(new IntakeCommand(intake, 0, false)));

    serializerRevButton.whenPressed(new SerializerRevCommand(serializer, 0.0, 0.3).alongWith(new IntakeCommand(intake, 0.5, true)));
    serializerRevButton.whenReleased(new SerializerRevCommand(serializer, 0, 0).alongWith(new IntakeCommand(intake, 0, false)));

    fireButton.whileHeld(new SerializerCommand(serializer, 0.5, -0.5));
    fireButton.whenReleased(new SerializerCommand(serializer, 0, 0));

    spitBallButton.whenPressed(new SpitTopBall(serializer, shooter));

    
 
  
    
/* ================================= DRIVE ================================= */

    leftStickButton.whenPressed(m_drivetrainSubsystem::zeroGyroscope); //Zero's the gyro to the robot's current direction

    lockOnSwitch.whileHeld(new JoystickDriveCommand(
      m_drivetrainSubsystem,
      () -> -modifyAxis(m_controller.getRawAxis(Constants.TRANSLATE_X_AXIS)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(-m_controller.getRawAxis(Constants.TRANSALTE_Y_AXIS)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(-m_drivetrainSubsystem.limelightOffset()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
).alongWith(new LimelightHoodLock(limelight,trajectoryTable,shooterHood, 3)));
    lockOnSwitch.whenReleased(new JoystickDriveCommand(
      m_drivetrainSubsystem,
      () -> -modifyAxis(m_controller.getRawAxis(Constants.TRANSLATE_X_AXIS)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(-m_controller.getRawAxis(Constants.TRANSALTE_Y_AXIS)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(-m_controller.getRawAxis(Constants.ROTATE_AXIS)) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
).alongWith(new LimelightHoodLock(limelight, trajectoryTable, shooterHood, 1)));



/* ================================= CLIMBER ================================= */
    //Actuation of the climber's pneumatic components via switches
    climbPneuButton1.whenPressed(new PneumaticsCommand1(pneumatics, Value.kForward));
    climbPneuButton1.whenReleased(new PneumaticsCommand1(pneumatics, Value.kReverse));

    
    singleRelockButton.whenPressed(new PneumaticsCommand2(pneumatics, Value.kForward));
    climbPneuButton2.whenPressed(new SingleLatchRelease(climber, pneumatics));
    climbPneuButton2.whenReleased(new ClimbCommand(climber));

    autoClimbButton.whenPressed(new AutoClimbSequence(climber, pneumatics, dioSub));

/* ================================= SHOOTER ================================= */
    shooterSpinup.whenPressed(new ShootCommand(shooter, 2600, true));
    shooterSpinup.whenReleased(new ShooterFinished(shooter));

    // shooterHoodUp.whileHeld(new ShooterHoodCommand(shooterHood), true);
    // shooterHoodUp.whenReleased(new ShooterHoodCommand(shooterHood), true);

   


    

  }

  /**\
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return new InstantCommand();

    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 3).setKinematics(m_drivetrainSubsystem.m_kinematics);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0 , new Rotation2d(0)),
      List.of(
               new Translation2d(3, 0),
               new Translation2d(3, -0.1),
               new Translation2d(0, -0.1)
               
               ),
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
      trajectoryConfig
      );

    PIDController xController = new PIDController(10.0, 0.0, 0);
    PIDController yController = new PIDController(10.0, 0.0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(0.0, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    trajectory,
     m_drivetrainSubsystem::getOdometryPose,
      m_drivetrainSubsystem.m_kinematics,
       xController,
        yController,
         thetaController,
          m_drivetrainSubsystem::setModuleStates,
           m_drivetrainSubsystem);

    return new SequentialCommandGroup(

      new InstantCommand(()-> m_drivetrainSubsystem.resetOdometry(trajectory.getInitialPose())),
      swerveControllerCommand,
      new InstantCommand(()-> m_drivetrainSubsystem.stop()).deadlineWith(new IntakeCommand(intake, 0, false))
    );

    
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
