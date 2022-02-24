// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
    //==================== Drivetrain Constants ====================
    public static final class Climber {
        public static final double CLIMB_SPEED = -0.55;
        public static final double HOLD_SPEED = -0.25; //motor power at which to hold the climber in rough position
        public static final double CLIMB_APPROACH_SPEED = -0.35;
    }
    public static final double DEADBAND_VALUE = 0.03; //controller deadband value
    public static final int TRANSLATE_X_AXIS = 4; //controller deadband value
    public static final int TRANSALTE_Y_AXIS = 1; //controller deadband value
    public static final int ROTATE_AXIS = 3; //controller deadband value
    public static final int GYRO_RECALIBRATE_BUTTON = 3; //recalibrate button on joysticks

    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.521; // The left-to-right distance between the drivetrain wheels, measured center-to-center
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.622; // The front-to-back distance between the drivetrain wheels, measured center-to-center

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 2;                           //Swerve FL Drive CAN ID
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 1;                          //Swerve FL Steer CAN ID
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 3;                         //Swerve FL CANCoder ID
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(346.72); // Steering offset. Measure as described on SDS Github

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 11;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 10; 
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 12; 
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(127.6); 

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 5;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 4;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 6;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(240.38);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 8;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 7;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 9;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(192.25);

    //==================== Climber Constants ====================

    public static final double CLIMBER_SENSOR_TIMEOUT_LOOPS = 10;

    public static final int CLIMBER_MANUAL_ROTATION_AXIS = 0;
    public static final int CLIMBER_DOUBLE_HAND_BUTTON = 3;
    public static final int CLIMBER_SINGLE_HAND_BUTTON = 1;
    public static final int CLIMBER_SINGLE_RELOCK_BUTTON = 7;
    public static final double CLIMBER_MAX_VOLTAGE = 12;
    public static final int CLIMBER_AUTO_BUTTON = 11;


    //DIO Pins on the RIO for climber sensors
    public static final int DOUBLEHAND_L_BOTTOM = 1;
    public static final int DOUBLEHAND_L_TOP = 0;
    public static final int DOUBLEHAND_R_BOTTOM = 2;
    public static final int DOUBLEHAND_R_TOP = 5;
    public static final int SINGLEHAND_L = 4;
    public static final int SINGLEHAND_R = 3;
    
    public static final class Serializer {
        public static final boolean serializerMotorInverted = false;
        public static final boolean serializerMotorSensorPhase = true;
        public static final int leftSerMotorport = 30;
        public static final int rightSerMotor = 31;
        public static final double peakOutput = 1;
        public static final double kP = 1.00;
        public static final double kI = 1.00;
        public static final double kD = 1.00;
        public static final double INTAKE_SPEED = 0.2;
        public static final double SHOOT_SPEED = 0.9;
    }

}   
