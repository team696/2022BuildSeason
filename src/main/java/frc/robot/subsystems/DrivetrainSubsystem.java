// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
// import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {
        Limelight limelight = new Limelight();
  /**
   * The maximum voltage that will be delivered to the drive motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 12.0;
  //  The formula for calculating the theoretical maximum velocity is:
  //  <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  * The maximum velocity of the robot in meters per second.
  //  * This is a measure of how fast the robot should be able to drive in a straight line.
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
          SdsModuleConfigurations.MK4_L1.getDriveReduction() *
          SdsModuleConfigurations.MK4_L1.getWheelDiameter() * Math.PI;


  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.
  private final AHRS m_navx = new AHRS(); // NavX connected over MXP
  private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(m_kinematics, new Rotation2d(0));

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  public ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public PIDController rotatePID;


  public DrivetrainSubsystem() {
        rotatePID = new PIDController(
                Constants.rotatePid_P,
                Constants.rotatePid_I,
                Constants.rotatePid_D);
        rotatePID.setTolerance(Constants.rotatePid_Tol);

        
    ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    // By default we will use Falcon 500s in standard configuration. But if you use a different configuration or motors
    // you MUST change it. If you do not, your code will crash on startup.

    m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            Mk4SwerveModuleHelper.GearRatio.L1,

            // This is the ID of the drive motor
            FRONT_LEFT_MODULE_DRIVE_MOTOR,

            // This is the ID of the steer motor
            FRONT_LEFT_MODULE_STEER_MOTOR,

            // This is the ID of the steer encoder
            FRONT_LEFT_MODULE_STEER_ENCODER,

            // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
            FRONT_LEFT_MODULE_STEER_OFFSET
    );

    // We will do the same for the other modules
    m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            Mk4SwerveModuleHelper.GearRatio.L1,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
            Mk4SwerveModuleHelper.GearRatio.L1,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
    );

    m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            Mk4SwerveModuleHelper.GearRatio.L1,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
    );
  }

  public double limelightOffset(){
          if(limelight.tx() > ((limelight.getDistance()/12) * Constants.limelightDeadbandCoefficient) ||
             limelight.tx() <  -((limelight.getDistance()/12) * Constants.limelightDeadbandCoefficient)){
        return rotatePID.calculate(limelight.tx(), 0);
          }
          else {
          return 0;
          }
}

public double joyControlUntilLock(double joystick){

        if(limelight.hasTarget() == 1){
                return limelightOffset();
        }
        else {
                return joystick;
        }
}

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {

   m_navx.zeroYaw();
  }

  public Rotation2d getGyroscopeRotation() {

   if (m_navx.isMagnetometerCalibrated()) {
     return Rotation2d.fromDegrees(m_navx.getFusedHeading());
   }
    return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
  }

  public void drive(ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  private SwerveModuleState getState(double velocity, double angle){
        return new SwerveModuleState(velocity, new Rotation2d(angle));
  }

  public Pose2d getOdometryPose() {
        return odometer.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
          odometer.resetPosition(pose, getGyroscopeRotation());
  }

  public void stop(){
        m_frontLeftModule.set(0, 0);
        m_frontRightModule.set(0, 0);
        m_backLeftModule.set(0, 0);
        m_backRightModule.set(0, 0);
  }
  

  public void setModuleStates(SwerveModuleState[] states){
        // SwerveModuleState[]  states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
    
        // final SwerveModuleState frontLeftState = getState(m_frontLeftModule.getDriveVelocity(), m_frontLeftModule.getSteerAngle());
        // final SwerveModuleState frontRightState = getState(m_frontRightModule.getDriveVelocity(), m_frontRightModule.getSteerAngle());
        // final SwerveModuleState backLeftState = getState(m_backLeftModule.getDriveVelocity(), m_backLeftModule.getSteerAngle());
        // final SwerveModuleState backRightState = getState(m_backRightModule.getDriveVelocity(), m_backRightModule.getSteerAngle());
    
        // odometer.update(getGyroscopeRotation(), frontLeftState, frontRightState, backLeftState, backRightState);
    
        m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
        m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
        m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
        m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
        
  }

  public void setModuleStates2(SwerveModuleState[] states){
        // SwerveModuleState[]  states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
    
        // final SwerveModuleState frontLeftState = getState(m_frontLeftModule.getDriveVelocity(), m_frontLeftModule.getSteerAngle());
        // final SwerveModuleState frontRightState = getState(m_frontRightModule.getDriveVelocity(), m_frontRightModule.getSteerAngle());
        // final SwerveModuleState backLeftState = getState(m_backLeftModule.getDriveVelocity(), m_backLeftModule.getSteerAngle());
        // final SwerveModuleState backRightState = getState(m_backRightModule.getDriveVelocity(), m_backRightModule.getSteerAngle());
    
        // odometer.update(getGyroscopeRotation(), frontLeftState, frontRightState, backLeftState, backRightState);
    
        m_frontLeftModule.set(states[0].speedMetersPerSecond, states[0].angle.getRadians());
        m_frontRightModule.set(states[1].speedMetersPerSecond, states[1].angle.getRadians());
        m_backLeftModule.set(states[2].speedMetersPerSecond, states[2].angle.getRadians());
        m_backRightModule.set(states[3].speedMetersPerSecond, states[3].angle.getRadians());
        
  }



  @Override
  public void periodic() {
        final SwerveModuleState frontLeftState = getState(m_frontLeftModule.getDriveVelocity(), m_frontLeftModule.getSteerAngle());
        final SwerveModuleState frontRightState = getState(m_frontRightModule.getDriveVelocity(), m_frontRightModule.getSteerAngle());
        final SwerveModuleState backLeftState = getState(m_backLeftModule.getDriveVelocity(), m_backLeftModule.getSteerAngle());
        final SwerveModuleState backRightState = getState(m_backRightModule.getDriveVelocity(), m_backRightModule.getSteerAngle());
    
        odometer.update(getGyroscopeRotation(), frontLeftState, frontRightState, backLeftState, backRightState);
    
//     SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
//     SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

//     final SwerveModuleState frontLeftState = getState(m_frontLeftModule.getDriveVelocity(), m_frontLeftModule.getSteerAngle());
//     final SwerveModuleState frontRightState = getState(m_frontRightModule.getDriveVelocity(), m_frontRightModule.getSteerAngle());
//     final SwerveModuleState backLeftState = getState(m_backLeftModule.getDriveVelocity(), m_backLeftModule.getSteerAngle());
//     final SwerveModuleState backRightState = getState(m_backRightModule.getDriveVelocity(), m_backRightModule.getSteerAngle());

//     odometer.update(getGyroscopeRotation(), frontLeftState, frontRightState, backLeftState, backRightState);

//     m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
//     m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
//     m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
//     m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
        // setModuleStates(m_kinematics.toSwerveModuleStates(m_chassisSpeeds));
 
  }

  
}
