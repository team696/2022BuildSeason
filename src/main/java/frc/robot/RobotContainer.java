// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.autos.FiveBall;
import frc.robot.autos.ThreeBall;
import frc.robot.autos.TwoBall;
import frc.robot.autos.TwoBallDisruptor;
import frc.robot.autos.TwoBallTest;
import frc.robot.commands.AutoClimbSequence;
import frc.robot.commands.AutoDoubleLatch;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ClimbCommand2;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LEDDefCommand;
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
import frc.robot.subsystems.LEDSub;
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
  /* TODO fix aiming pid, fix hood pid, make sure auto works, driver testing, auto recenter */

  public final Serializer serializer = new Serializer();
  public final Climber climber = new Climber();
  public  final Pneumatics pneumatics = new Pneumatics();
  public final Shooter shooter = new Shooter();
  public final DIOSub dioSub = new DIOSub();
  public final ShooterHood shooterHood = new ShooterHood();
  public final Intake intake = new Intake();
  public final Limelight limelight = new Limelight();
  public final TrajectoryTable trajectoryTable = new TrajectoryTable();
  public  final Swerve s_Swerve = new Swerve();
  public final LEDSub ledSub = new LEDSub();
  private final Joystick m_controller = new Joystick(0);
  public  static final Joystick controlPanel = new Joystick(2);
  public static final Joystick singleController = new Joystick(3);

  /**Button on the left driver station joystick. */
  private final JoystickButton leftStickButton = new JoystickButton(m_controller,   Constants.GYRO_RECALIBRATE_BUTTON  );

  /**Button labeled "LATCHES" on the operator panel.*/
  private final JoystickButton climbPneuButton1 = new JoystickButton(controlPanel, Constants.CLIMBER_DOUBLE_HAND_BUTTON);

  /**Button labeled "HOOKS" on the operator panel. */
  private final JoystickButton climbPneuButton2 = new JoystickButton(controlPanel, Constants.CLIMBER_SINGLE_HAND_BUTTON);

  /** Switch labeled "CLIMB" on the operator panel when it is in the up position. */
  private final JoystickButton autoClimbButton = new JoystickButton(controlPanel, Constants.CLIMBER_AUTO_BUTTON);

  /**Button labeled "DEPLOY" on the operator panel. */
  private final JoystickButton serializerForButton = new JoystickButton(controlPanel, Constants.INTAKE_DEPLOY_BUTTON );

  /**Button labeled "SPIN UP" on the operator panel. */
  private final JoystickButton shooterSpinup = new JoystickButton(controlPanel, Constants.SPINUP_SWITCH);

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

  /** Silver switch labeled "ARM" (???????).*/
  private final JoystickButton armButton = new JoystickButton(controlPanel, 4);

  private final JoystickButton serializerRevButton = new JoystickButton(controlPanel, 20);

  private final JoystickButton flashbangButton = new JoystickButton(controlPanel, 30);

  private final JoystickButton testButton  = new JoystickButton(controlPanel, 15);

  /** Button */
  private final JoystickButton singleRelockButton = new JoystickButton(controlPanel , Constants.CLIMBER_SINGLE_RELOCK_BUTTON);

  private final JoystickButton shooterHoodUp = new JoystickButton(controlPanel, 0);

  private final JoystickButton lockOnButton_SC = new  JoystickButton(singleController, 5);/* Shoulder Button  */
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
  private final SequentialCommandGroup threeBall = new ThreeBall(s_Swerve, limelight, shooter, serializer, shooterHood, trajectoryTable, intake, m_controller);
  private final SequentialCommandGroup twoBallTest = new TwoBallTest(s_Swerve, limelight, shooter, serializer, shooterHood, trajectoryTable, intake, m_controller);
  private final SequentialCommandGroup twoBallDisruptor = new TwoBallDisruptor(s_Swerve, limelight, shooter, serializer, shooterHood, trajectoryTable, intake, m_controller);

    // public SendableChooser<Command> colorChooser = new SendableChooser<>();
    // public  final Command  blueAlliance  = new CheckBlueAlliance(serializer, shooter);
    // public final Command  redAlliance = new CheckRedAlliance(serializer, shooter);


     boolean fieldRelative = true;
    boolean openLoop = true;
  
  static public boolean isShooting = false;
  public static  double shootSpeed = 3000;

  private final int translationAxis = 1;
  private final int strafeAxis =  0;
  private final int rotationAxis = 2;

 
 




  // public double climberManual_SC(){
  //   return (singleController.getRawAxis(3) - singleController.getRawAxis(2))/2;
    
  // }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // serializer = new Serializer();
    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, m_controller, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));
    climber.setDefaultCommand(new ClimbCommand(climber));
    shooterHood.setDefaultCommand(new ShooterHoodCommand(shooterHood, 0));
    ledSub.setDefaultCommand(new LEDDefCommand(ledSub));
    // serializer.setDefaultCommand(new CheckBlueAlliance(serializer, shooter).andThen(new SpitTopBall(serializer, shooter)));
  
    m_chooser.setDefaultOption("Five Ball Auto", fiveBall);
    m_chooser.addOption("Two Ball Auto ", twoBall);
    m_chooser.addOption("Three Ball Auto", threeBall );
    m_chooser.addOption("TWO BALL TEST", twoBallTest);
    m_chooser.addOption("Two Ball Disruptor", twoBallDisruptor);
    // colorChooser.setDefaultOption("Red Alliance", redAlliance);
    // colorChooser.addOption("Blue Alliance ", blueAlliance);

    SmartDashboard.putData(m_chooser);
    // SmartDashboard.putData(colorChooser);
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

    serializerForButton.whenHeld(
      new SerializerCommand(serializer, 0.2, -0.6, shooter, 0.4 ).alongWith(
        new IntakeCommand(intake, -0.5, true)));

    dropBallButton.whenHeld(
      new SerializerRevCommand(serializer, 0.0, 0.3).alongWith(
        new IntakeCommand(intake, 0.8, true)));

    spitBallButton.whenPressed(
      new SpitTopBall(serializer, shooter));

    fireButton.whenPressed(
      new FireCommand(serializer));

    intakeButtonDown.whenHeld(
      new SerializerCommand(serializer, 0.3, -0.3, shooter, 0.2));
    
/* ================================= DRIVE ================================= */
    // leftStickButton.whileHeld(new InstantCommand(() -> s_Swerve.zeroGyro())); 
    leftStickButton.whileHeld((new TeleopSwerve(s_Swerve, m_controller, translationAxis, strafeAxis, rotationAxis, false, openLoop)));
    testButton.whenPressed(new InstantCommand(() -> s_Swerve.zeroGyro()));

    lockOnSwitch.whileHeld(
                new LimelightLockSwerve(s_Swerve, m_controller, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop).alongWith(
                new LimelightHoodLock(limelight,trajectoryTable,shooterHood, 3, ledSub, false)));
    lockOnSwitch.whenReleased(
                new TeleopSwerve(s_Swerve, m_controller, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop).alongWith(
                new LimelightHoodLock(limelight, trajectoryTable, shooterHood, 1, ledSub, true )/* .alongWith(
                new ShooterHoodCommand(shooterHood, 30) */));

/* ================================= CLIMBER ================================= */
    climbPneuButton1.toggleWhenPressed(
      new PneumaticsCommand1(pneumatics, Value.kForward));

    singleRelockButton.whenPressed(
      new PneumaticsCommand2(pneumatics, Value.kForward));
    climbPneuButton2.whenPressed(
      new SingleLatchRelease(climber, pneumatics));
    climbPneuButton2.whenReleased(
      new ClimbCommand(climber));

    autoClimbButton.whenPressed(
      new AutoClimbSequence(climber, pneumatics, dioSub));
    autoClimbButton.whenReleased(
      new ClimbCommand(climber));

    armButton.whenPressed(
      new AutoDoubleLatch(pneumatics, dioSub).deadlineWith(
      new ClimbCommand2(climber).alongWith(
      new TeleopSwerveSlow(s_Swerve, m_controller, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop))),true);

    armButton.whenReleased(
      new ClimbCommand2(climber).alongWith(
      new TeleopSwerve(s_Swerve, m_controller, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop)), true);
    
/* ================================= SHOOTER ================================= */
    shooterSpinup.whileHeld(
      new ShootCommand(shooter, /* shootSpeed, */ true));
    shooterSpinup.whenReleased(
      new ShooterFinished(shooter));

    

    

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
   
return m_chooser.getSelected();   
    
  }

 
  
}
