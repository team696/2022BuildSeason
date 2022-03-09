// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import java.nio.file.FileSystem;
import java.nio.file.Path;
import java.util.List;

import javax.swing.plaf.basic.BasicArrowButton;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.AutoClimbSequence;
import frc.robot.commands.AutoClimbSequenceNew;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.ClimbCommand;

import frc.robot.commands.IntakeCommand;
import frc.robot.commands.DIOTest;
import frc.robot.commands.FireCommand;
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
  public static final Joystick singleController = new Joystick(3);

  private final JoystickButton leftStickButton = new JoystickButton(m_controller,  Constants.GYRO_RECALIBRATE_BUTTON) ;
  private final JoystickButton climbPneuButton1 = new JoystickButton(controlPanel, Constants.CLIMBER_DOUBLE_HAND_BUTTON);
  private final JoystickButton climbPneuButton2 = new JoystickButton(controlPanel, Constants.CLIMBER_SINGLE_HAND_BUTTON);
  private final JoystickButton singleRelockButton = new JoystickButton(controlPanel , Constants.CLIMBER_SINGLE_RELOCK_BUTTON);
  private final JoystickButton autoClimbButton = new JoystickButton(controlPanel, Constants.CLIMBER_AUTO_BUTTON);
  private final JoystickButton serializerForButton = new JoystickButton(controlPanel, 11 );
  private final JoystickButton serializerRevButton = new JoystickButton(controlPanel, 20);
  private final JoystickButton shooterSpinup = new JoystickButton(controlPanel, 1);
  private final JoystickButton shooterHoodUp = new JoystickButton(controlPanel, 0);
  private final JoystickButton spitBallButton = new JoystickButton(controlPanel, 9);
  private final JoystickButton dropBallButton = new JoystickButton(controlPanel, 12);
  private final JoystickButton lockOnSwitch = new JoystickButton(m_controller, 4);
  private final JoystickButton fireButton = new JoystickButton(controlPanel, 3);
  private final JoystickButton hoodControlButton = new JoystickButton(controlPanel, 10);
  private final JoystickButton emergencyClimbManual = new JoystickButton(controlPanel, 2);
  private final JoystickButton intakeButtonUp = new JoystickButton(controlPanel, 13);
  private final JoystickButton intakeButtonDown = new JoystickButton(controlPanel, 14);
  


  private final JoystickButton lockOnButton_SC = new  JoystickButton(singleController, 5); /* Shoulder Button  */
  private final JoystickButton spinupButton_SC = new JoystickButton(singleController, 6); /* Other shoulder button */
  private final JoystickButton intakeButton_SC = new JoystickButton(singleController, 1); /* A main button  */
  private final JoystickButton intakeRevButton_SC = new JoystickButton(singleController, 2); /* A main button  */
  private final JoystickButton fireButton_SC = new JoystickButton(singleController, 3); /* A main button  */
  private final JoystickButton autoClimbButton_SC = new JoystickButton(singleController, 7); /* Plus button  */
  private final POVButton toggleDoubleLatch_SC = new POVButton(singleController, 0); /* Up */
  private final POVButton singleLatchRelease_SC = new POVButton(singleController, 90); /* Left */
  private final POVButton singleLatchRelock_SC = new POVButton(singleController, 270); /* Right */
  private final JoystickButton serializeButton_SC = new JoystickButton(singleController, 4); /* A main button */
  private final JoystickButton resetGyro_SC = new JoystickButton(singleController, 9); 

  





  
  static public boolean isShooting = false;
  public static  double shootSpeed = 3000;



  // public double climberManual_SC(){
  //   return (singleController.getRawAxis(3) - singleController.getRawAxis(2))/2;
    
  // }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // climber.setDefaultCommand(new ClimbCommand(climber));
    shooterHood.setDefaultCommand(new ShooterHoodCommand(shooterHood));
   /* TODO SWITCH FOR DRIVERSTATION  */
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
//     m_drivetrainSubsystem.setDefaultCommand(new JoystickDriveCommand(
//       m_drivetrainSubsystem,
//       () -> -modifyAxis(singleController.getRawAxis(1)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
//       () -> -modifyAxis(singleController.getRawAxis(0)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
//       () -> -modifyAxis(singleController.getRawAxis(4)) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
// ));



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

    serializerForButton.whenHeld(new SerializerCommand(serializer, 0.2, -0.6).alongWith(new IntakeCommand(intake, -0.5, true)));

    dropBallButton.whenHeld(new SerializerRevCommand(serializer, 0.0, 0.3).alongWith(new IntakeCommand(intake, 0.5, true)));


    spitBallButton.whenPressed(new SpitTopBall(serializer, shooter));


    fireButton.whenPressed(new FireCommand(serializer));

    intakeButtonDown.whenHeld(new SerializerCommand(serializer, 0.3, -0.3));
    
 
  
    
/* ================================= DRIVE ================================= */
 /* TODO ADD BACK FOR DRIVERSTATION  */
    leftStickButton.whenPressed(m_drivetrainSubsystem::zeroGyroscope); 
// if(hoodControlButton.getAsBoolean()){
    lockOnSwitch.whileHeld(new JoystickDriveCommand(
      m_drivetrainSubsystem,
      () -> -modifyAxis(m_controller.getRawAxis(Constants.TRANSLATE_X_AXIS)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(-m_controller.getRawAxis(Constants.TRANSALTE_Y_AXIS)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis( -m_drivetrainSubsystem.joyControlUntilLock(m_controller.getRawAxis(Constants.ROTATE_AXIS))) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
).alongWith(new LimelightHoodLock(limelight,trajectoryTable,shooterHood, 3)));
    lockOnSwitch.whenReleased(new JoystickDriveCommand(
      m_drivetrainSubsystem,
      () -> -modifyAxis(m_controller.getRawAxis(Constants.TRANSLATE_X_AXIS)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(-m_controller.getRawAxis(Constants.TRANSALTE_Y_AXIS)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(-m_controller.getRawAxis(Constants.ROTATE_AXIS)) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
).alongWith(new LimelightHoodLock(limelight, trajectoryTable, shooterHood, 1)));
// }
  hoodControlButton.cancelWhenActive(new ShooterHoodCommand(shooterHood));
hoodControlButton.whenReleased(new ShooterHoodCommand(shooterHood));
 

/* ================================= CLIMBER ================================= */
    //Actuation of the climber's pneumatic components via switches
    climbPneuButton1.toggleWhenPressed(new PneumaticsCommand1(pneumatics, Value.kForward));

    
    singleRelockButton.whenPressed(new PneumaticsCommand2(pneumatics, Value.kForward));
    climbPneuButton2.whenPressed(new SingleLatchRelease(climber, pneumatics));
    climbPneuButton2.whenReleased(new ClimbCommand(climber));

    if(!emergencyClimbManual.get()){
    autoClimbButton.whenHeld(new AutoClimbSequence(climber, pneumatics, dioSub));
    }

    emergencyClimbManual.whenHeld(new ClimbCommand(climber));

/* ================================= SHOOTER ================================= */
    shooterSpinup.whenPressed(new ShootCommand(shooter, /* shootSpeed, */ true));
    shooterSpinup.whenReleased(new ShooterFinished(shooter));

    

    // shooterHoodUp.whileHeld(new ShooterHoodCommand(shooterHood), true);
    // shooterHoodUp.whenReleased(new ShooterHoodCommand(shooterHood), true);

/* ================================= SINGLE CONTROLLER ================================= */
/* TODO REMOVE FOR DRIVERSTATION  */
// resetGyro_SC.whenPressed(m_drivetrainSubsystem::zeroGyroscope); //Zero's the gyro to the robot's current direction

// lockOnButton_SC.toggleWhenPressed(new JoystickDriveCommand(
//   m_drivetrainSubsystem,
//   () -> -modifyAxis(singleController.getRawAxis(1)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
//   () -> -modifyAxis(singleController.getRawAxis(0)) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
//   () -> -modifyAxis(/* -m_drivetrainSubsystem.limelightOffset() */ -m_drivetrainSubsystem.joyControlUntilLock(-singleController.getRawAxis(4))) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
// ).alongWith(new LimelightHoodLock(limelight,trajectoryTable,shooterHood, 3)));


// spinupButton_SC.whenPressed(new ShootCommand(shooter, true));
// spinupButton_SC.whenReleased(new ShooterFinished(shooter));

// intakeButton_SC.whenHeld(new SerializerCommand(serializer, 0.2, -0.6).alongWith(new IntakeCommand(intake, -0.5, true)));
// intakeRevButton_SC.whenHeld(new SerializerRevCommand(serializer, 0.0, 0.3).alongWith(new IntakeCommand(intake, 0.5, true)));

// fireButton_SC.whenPressed(new FireCommand(serializer));

// autoClimbButton_SC.whenPressed(new AutoClimbSequence(climber, pneumatics, dioSub));

// toggleDoubleLatch_SC.toggleWhenPressed(new PneumaticsCommand1(pneumatics, Value.kForward));

// singleLatchRelease_SC.whenPressed(new SingleLatchRelease(climber, pneumatics));
// singleLatchRelease_SC.whenReleased(new ClimbCommand(climber));

// singleLatchRelock_SC.whenPressed(new PneumaticsCommand2(pneumatics, Value.kForward));

// serializeButton_SC.whileHeld(new SerializerCommand(serializer, 0.2, -0.4));
// serializeButton_SC.whenReleased(new SerializerCommand(serializer, 0, 0));

   
   


    

  }

  /**\
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    String trajectoryJSON = "Paths/Step1.wpilib.json";

    Path trajectoryPath =  Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    // An ExampleCommand will run in autonomous
    // return new InstantCommand();
    Command lockAndShoot = new ParallelCommandGroup(new AutoShoot(limelight, shooter, serializer, shooterHood, trajectoryTable).deadlineWith(new JoystickDriveCommand(
      m_drivetrainSubsystem,
      () -> -modifyAxis(0) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(0) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(-m_drivetrainSubsystem.limelightOffset()/*  -m_drivetrainSubsystem.joyControlUntilLock(-singleController.getRawAxis(4) */) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    )));

    Command lockAndShoot2 = new ParallelCommandGroup(new AutoShoot(limelight, shooter, serializer, shooterHood, trajectoryTable).deadlineWith(new JoystickDriveCommand(
      m_drivetrainSubsystem,
      () -> -modifyAxis(0) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(0) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(-m_drivetrainSubsystem.limelightOffset()/*  -m_drivetrainSubsystem.joyControlUntilLock(-singleController.getRawAxis(4) */) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    )));

    Command lockAndShoot3 = new ParallelCommandGroup(new AutoShoot(limelight, shooter, serializer, shooterHood, trajectoryTable).deadlineWith(new JoystickDriveCommand(
      m_drivetrainSubsystem,
      () -> -modifyAxis(0) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(0) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(-m_drivetrainSubsystem.limelightOffset()/*  -m_drivetrainSubsystem.joyControlUntilLock(-singleController.getRawAxis(4) */) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
    )));



    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND, 3).setKinematics(m_drivetrainSubsystem.m_kinematics);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0 , new Rotation2d(0)),
      List.of(
              //  new Translation2d(1, 0.5),
               new Translation2d(0.5, 0.01),
               new Translation2d(-1.5, -3)
               ),
      new Pose2d(-1.5, -4, Rotation2d.fromDegrees(300)), 
      trajectoryConfig
      );

    Trajectory trajectoryDos= TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0 , new Rotation2d(0)),
      List.of(
              new Translation2d(1.5, 0.01)      
              ),
      new Pose2d(1, 0, Rotation2d.fromDegrees(0)), 
      trajectoryConfig
      );

      Trajectory trajectoryTres= TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0 , new Rotation2d(0)),
      List.of(
              new Translation2d(-0.2,-7)      
              ),
      new Pose2d(0.5, -13, Rotation2d.fromDegrees(300)), 
      trajectoryConfig
      );

      // Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

    PIDController xController = new PIDController(0.0, 0.0, 01);
    PIDController yController = new PIDController(0.0, 0.0, 01);
    ProfiledPIDController thetaController = new ProfiledPIDController(7.0, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    trajectory,
     m_drivetrainSubsystem::getOdometryPose,
      m_drivetrainSubsystem.m_kinematics,
       new PIDController(5, 0, 0),
        new PIDController(5, 0, 0),
         thetaController,
          m_drivetrainSubsystem::setModuleStates2,
           m_drivetrainSubsystem);


           SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(
            trajectoryDos,
             m_drivetrainSubsystem::getOdometryPose,
              m_drivetrainSubsystem.m_kinematics,
               new PIDController(2, 0, 0),
                new PIDController(2, 0, 0),
                 thetaController,
                  m_drivetrainSubsystem::setModuleStates2,
                   m_drivetrainSubsystem);
       
          SwerveControllerCommand swerveControllerCommand3 = new SwerveControllerCommand(
            trajectoryTres,
             m_drivetrainSubsystem::getOdometryPose,
              m_drivetrainSubsystem.m_kinematics,
               new PIDController(4, 0, 0),
                new PIDController(4, 0, 0),
                 thetaController,
                  m_drivetrainSubsystem::setModuleStates2,
                   m_drivetrainSubsystem);
    

    return new SequentialCommandGroup(  /* AUTO SHIT ACTUALLY GOES HERE  also TODO MAYBE MAKE INDIVIDUAL COMMAND GROUPS FOR EACH FIELD POSITION */
      
       new InstantCommand(()-> m_drivetrainSubsystem.resetOdometry(trajectory.getInitialPose())),
         swerveControllerCommand2.deadlineWith(new IntakeCommand(intake, -0.5, true)),
          new InstantCommand(()-> m_drivetrainSubsystem.stop()).deadlineWith(new IntakeCommand(intake, 0, false)) ,
            lockAndShoot2,
              swerveControllerCommand.deadlineWith(new IntakeCommand(intake, -0.5, true)),
               new InstantCommand(()-> m_drivetrainSubsystem.stop()).deadlineWith(new IntakeCommand(intake, -0.5, true)) ,
                lockAndShoot,
                  swerveControllerCommand3.deadlineWith(new IntakeCommand(intake, -0.5, true)),
                    lockAndShoot3.deadlineWith(new IntakeCommand(intake, -0.5, true ))
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
