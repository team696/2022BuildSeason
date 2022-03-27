// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.nio.file.Path;
import java.util.List;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.autos.FiveBall;
import frc.robot.autos.TwoBall;
import frc.robot.commands.AutoClimbSequence;
import frc.robot.commands.AutoDoubleLatch;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ClimbCommand2;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeDelay;
import frc.robot.commands.FireCommand;
import frc.robot.commands.LimelightHoodLock;
import frc.robot.commands.LimelightLockSwerve;
import frc.robot.commands.PneumaticsCommand1;
import frc.robot.commands.PneumaticsCommand2;
import frc.robot.commands.SerializerCommand;
import frc.robot.commands.SerializerRevCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShooterFinished;
import frc.robot.commands.ShooterHoodCommand;
import frc.robot.commands.SingleLatchRelease;
import frc.robot.commands.SpitTopBall;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TeleopSwerveSlow;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DIOSub;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Serializer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TrajectoryTable;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public final Serializer serializer = new Serializer();
  public final Climber climber = new Climber();
  public  final Pneumatics pneumatics = new Pneumatics();
  public final Shooter shooter = new Shooter();
  public final DIOSub dioSub = new DIOSub();
  public final ShooterHood shooterHood = new ShooterHood();
  public final Intake intake = new Intake();
  public final Limelight limelight = new Limelight();
  public final TrajectoryTable trajectoryTable = new TrajectoryTable();
  private final Swerve s_Swerve = new Swerve();
  private final Joystick m_controller = new Joystick(0);
  public  static final Joystick controlPanel = new Joystick(2);
  public static final Joystick singleController = new Joystick(3);

  /**Button on the left driver station joystick. */
  private final JoystickButton leftStickButton = new JoystickButton(m_controller,   Constants.GYRO_RECALIBRATE_BUTTON  );

  /**Button labeled "LATCHES" on the operator panel.*/
  private final JoystickButton climbPneuButton1 = new JoystickButton(controlPanel, Constants.CLIMBER_DOUBLE_HAND_BUTTON);

  /**Button labeled "HOOKS" on the operator panel. */
  private final JoystickButton climbPneuButton2 = new JoystickButton(controlPanel, Constants.CLIMBER_SINGLE_HAND_BUTTON);

  private final JoystickButton singleRelockButton = new JoystickButton(controlPanel , Constants.CLIMBER_SINGLE_RELOCK_BUTTON);

  /** Switch labeled "CLIMB" on the operator panel when it is in the up position. */
  private final JoystickButton autoClimbButton = new JoystickButton(controlPanel, Constants.CLIMBER_AUTO_BUTTON);

  /**Button labeled "DEPLOY" on the operator panel. */
  private final JoystickButton serializerForButton = new JoystickButton(controlPanel, Constants.INTAKE_DEPLOY_BUTTON );

  private final JoystickButton serializerRevButton = new JoystickButton(controlPanel, 20);

  /**Button labeled "SPIN UP" on the operator panel. */
  private final JoystickButton shooterSpinup = new JoystickButton(controlPanel, Constants.SPINUP_SWITCH);

  private final JoystickButton shooterHoodUp = new JoystickButton(controlPanel, 0);

  /**Second unlabeled button to the left of the "DEPLOY" button on the operator panel. */
  private final JoystickButton spitBallButton = new JoystickButton(controlPanel, Constants.SPIT_BALL_BUTTON);

  /** First unlabeled button to the left of the "DEPLOY" button on the operator panel.   */
  private final JoystickButton dropBallButton = new JoystickButton(controlPanel, Constants.DROP_BALL_BUTTON);

  /** Button on the right driver station joystick. */
  private final JoystickButton lockOnSwitch = new JoystickButton(m_controller, Constants.JOYSTICK_RIGHT_BUTTON);

  /** Button labeled "SHOOT" on the operator panel. */
  private final JoystickButton fireButton = new JoystickButton(controlPanel, Constants.FIRE_BUTTON);

  /** Silver switch labeled "MAN" and "AUTO" on the operator panel. */
  private final JoystickButton hoodControlButton = new JoystickButton(controlPanel, Constants.HOOD_CONTROL_SWITCH);

  /** Covered switch on the operator panel. */
  private final JoystickButton emergencyClimbManual = new JoystickButton(controlPanel, Constants.EMERGENCY_CLIMB_SWITCH);

  /** Switch labeled "INTAKE" when it is set to the up position(toggle switch). */
  private final JoystickButton intakeButtonUp = new JoystickButton(controlPanel, Constants.INTAKE_SWITCH_UP);

  /**Switch labeled "INTAKE" when it is set to the down position (momentary switch). */
  private final JoystickButton intakeButtonDown = new JoystickButton(controlPanel, Constants.INTAKE_SWITCH_DOWN);

  private final JoystickButton flashbangButton = new JoystickButton(controlPanel, 30);

  private final JoystickButton testButton  = new JoystickButton(controlPanel, 15);

  /** Silver switch labeled "ARM" (???????).*/
  private final JoystickButton armButton = new JoystickButton(controlPanel, 4);
  


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

    public SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final SequentialCommandGroup fiveBall = new FiveBall(s_Swerve, limelight, shooter, serializer, shooterHood, trajectoryTable, intake, m_controller);
  private final SequentialCommandGroup twoBall = new TwoBall(s_Swerve, limelight, shooter, serializer, shooterHood, trajectoryTable, intake, m_controller);




     boolean fieldRelative = true;
    boolean openLoop = true;
  
  static public boolean isShooting = false;
  public static  double shootSpeed = 3000;

  private final int translationAxis = 1;
  private final int strafeAxis =  0;
  private final int rotationAxis = 2;

 
  /* Subsystems */




  // public double climberManual_SC(){
  //   return (singleController.getRawAxis(3) - singleController.getRawAxis(2))/2;
    
  // }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
 
    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, m_controller, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));

    climber.setDefaultCommand(new ClimbCommand(climber));
    shooterHood.setDefaultCommand(new ShooterHoodCommand(shooterHood, 0));
  
    m_chooser.setDefaultOption("Five Ball Auto", fiveBall);
    m_chooser.addOption("Two Ball Auto ", twoBall);

    SmartDashboard.putData(m_chooser);


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

    serializerForButton.whenHeld(new SerializerCommand(serializer, 0.2, -0.6).alongWith(new IntakeCommand(intake, -0.6, true)));

    dropBallButton.whenHeld(new SerializerRevCommand(serializer, 0.0, 0.3).alongWith(new IntakeCommand(intake, 0.8, true)));


    spitBallButton.whenPressed(new SpitTopBall(serializer, shooter));


    fireButton.whenPressed(new FireCommand(serializer));

    intakeButtonDown.whenHeld(new SerializerCommand(serializer, 0.3, -0.3));
    
 
  
    
/* ================================= DRIVE ================================= */
    leftStickButton.whenPressed(new InstantCommand(() -> s_Swerve.zeroGyro())); 


    lockOnSwitch.whileHeld(new LimelightLockSwerve(s_Swerve, m_controller, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop).alongWith(new LimelightHoodLock(limelight,trajectoryTable,shooterHood, 3)));
    lockOnSwitch.whenReleased(new TeleopSwerve(s_Swerve, m_controller, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop).alongWith(new LimelightHoodLock(limelight, trajectoryTable, shooterHood, 1)));

//   hoodControlButton.cancelWhenActive(new ShooterHoodCommand(shooterHood));
// hoodControlButton.whenReleased(new ShooterHoodCommand(shooterHood));
// hoodControlButton.whileHeld(new ShooterHoodCommand(shooterHood, 0.6));
// hoodControlButton.whenReleased(new ShooterHoodCommand(shooterHood, 1.2));

/* ================================= CLIMBER ================================= */
    climbPneuButton1.toggleWhenPressed(new PneumaticsCommand1(pneumatics, Value.kForward));

    
    singleRelockButton.whenPressed(new PneumaticsCommand2(pneumatics, Value.kForward));
    climbPneuButton2.whenPressed(new SingleLatchRelease(climber, pneumatics));
    climbPneuButton2.whenReleased(new ClimbCommand(climber));



    autoClimbButton.whenPressed(new AutoClimbSequence(climber, pneumatics, dioSub));
    autoClimbButton.whenReleased(new ClimbCommand(climber));

    armButton.whenPressed(
      new AutoDoubleLatch(pneumatics, dioSub).deadlineWith(
      new ClimbCommand2(climber).alongWith(
      new TeleopSwerveSlow(s_Swerve, m_controller, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop))),true);

      armButton.whenReleased(
        new ClimbCommand2(climber).alongWith(
        new TeleopSwerve(s_Swerve, m_controller, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop)), true);
    


/* ================================= SHOOTER ================================= */
    shooterSpinup.whenPressed(new ShootCommand(shooter, /* shootSpeed, */ true));
    shooterSpinup.whenReleased(new ShooterFinished(shooter));

    

    

/* ================================= SINGLE CONTROLLER ================================= */
/* TODO REMOVE FOR DRIVERSTATION  */
// resetGyro_SC.whenPressed(m_drivetrainSubsystem::zeroGyroscope); //Zero's the gyro to the robot's current direction


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
   
    // return new FiveBall(s_Swerve, limelight, shooter, serializer, shooterHood, trajectoryTable, intake, m_controller);
return m_chooser.getSelected();   
    
  }

 
  
}
