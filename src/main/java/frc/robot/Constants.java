// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {

    public static final double stickDeadband = 0.06;

    public static final class Swerve {
        public static final int pigeonID = 1;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */ 
        public static final double trackWidth = /* Units.inchesToMeters(21.73) */ 0.521;
        public static final double wheelBase = /* Units.inchesToMeters(21.73) */    0.622;
        public static final double wheelDiameter = Units.inchesToMeters(3.94);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.5;
        public static final double closedLoopRamp = 0.5;

        public static final double driveGearRatio = (8.14 / 1.0); //6.86:1
        public static final double angleGearRatio = (12.8 / 1.0); //12.8:1

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.6;
        public static final double angleKI = 0.0;
        public static final double angleKD = 12.0;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.10;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = (0.667 / 12); //divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.5; //meters per second
        public static final double maxAngularVelocity = 11.5;

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Motor Inverts */
        public static final boolean driveMotorInvert = false;
        public static final boolean angleMotorInvert = false;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 3;
            public static final double angleOffset =  141.2   /* Math.toRadians(141.2) */  /* Math.toRadians(40) */;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 11;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 12;
            public static final double angleOffset =   81 /* Math.toRadians(81) */ /*  Math.toRadians(100) */;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 6;
            public static final double angleOffset =  79.36 /* Math.toRadians(74.1) */  /* -Math.toRadians(254.5) */;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 9;
            public static final double angleOffset =  152.3 /*   Math.toRadians(152.3); */ /* -Math.toRadians(333) */;
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 0.15;
    
        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
      }
    //==================== Drivetrain Constants ====================
    public static final class Climber {
        public static final double CLIMB_SPEED = -0.55;
        public static final double HOLD_SPEED = -0.25; //motor power at which to hold the climber in rough position
        public static final double CLIMB_APPROACH_SPEED = -0.35;
    }   
     public static final int GYRO_RECALIBRATE_BUTTON = 1; //recalibrate button on joysticks

   
    public static final double limelightDeadbandCoefficient = 0.055
    ;
    public static final double rotatePid_P = /* 0.025 */ 0.02;
    public static final double rotatePid_I = /* 0.0002 */ 0.0005;
    public static final double rotatePid_D = /* 0.003 */0.00375;
    public static final double rotatePid_Tol = 1;

   
    // kMaxAngularSpeedRadiansPerSecond
    // kMaxAngularAccelerationRadiansPerSecondSquared
   
 //==================== Operator Constants ====================
    public static final double CLIMBER_SENSOR_TIMEOUT_LOOPS = 10;

    public static final int CLIMBER_MANUAL_ROTATION_AXIS = 0;
    public static final int CLIMBER_DOUBLE_HAND_BUTTON = 7;
    public static final int CLIMBER_SINGLE_HAND_BUTTON = 8;
    public static final int CLIMBER_SINGLE_RELOCK_BUTTON = 13;
    public static final double CLIMBER_MAX_VOLTAGE = 12;
    public static final int CLIMBER_AUTO_BUTTON = 6;
    public static final int INTAKE_DEPLOY_BUTTON = 11;
    public static final int SPINUP_SWITCH = 1;
    public static final int SPIT_BALL_BUTTON = 9;
    public static final int DROP_BALL_BUTTON = 12;
    public static final int JOYSTICK_RIGHT_BUTTON = 2;
    public static final int FIRE_BUTTON = 3;
    public static final int HOOD_CONTROL_SWITCH = 10;
    public static final int EMERGENCY_CLIMB_SWITCH = 2;
    public static final int INTAKE_SWITCH_UP = 13;
    public static final int INTAKE_SWITCH_DOWN = 14;


 //==================== Climber Constants ====================
    //DIO Pins on the RIO for climber sensors
    public static final int DOUBLEHAND_L_BOTTOM = 2; /* 1 */
    public static final int DOUBLEHAND_L_TOP = 0;  /* 0 */
    public static final int DOUBLEHAND_R_BOTTOM = 5; /* 4 */
    public static final int DOUBLEHAND_R_TOP = 4; /* 5 */
    public static final int SINGLEHAND_L = 1; /* 2 */
    public static final int SINGLEHAND_R = 3; /* 3 */
    
    public static final class Serializer {
        public static final boolean serializerMotorInverted = false;
        public static final boolean serializerMotorSensorPhase = true;
        public static final int leftSerMotorport = 30;
        public static final int rightSerMotor = 3;
        public static final double peakOutput = 1;
        public static final double kP = 1.00;
        public static final double kI = 1.00;
        public static final double kD = 1.00;
        public static final double INTAKE_SPEED = 0.2;
        public static final double SHOOT_SPEED = 0.8;
    }
    public static final class LimelightConstants{
                                                               
        public static final double limelightDegrees = 39;  
        public static final double limelightHeight = 24; 
        public static final double goalHeight = 104; 

    }


public static final class Shooter {
        public static final int hoodAutoButton = 9;
        public static final int hoodAxis = 1;
        public static final double hoodPeakOutput = 0.7;
        public static final double hoodLimitFor = 130;
        public static final double hoodLimitRev = 5;
        public static final boolean hoodSoftLimitForward = false;
        public static final boolean hoodSoftLimitReverse = false;
        public static final double kP = 0.5;  /* TODO TUNE THIS PID  */
        public static final double kI = 0.0005;
        public static final double kD = 0.0000;
        public static final double kTolerance = 10;
        public static final int hoodMotorPort = 60;
    }
}
