package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    private Limelight limelight;
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    // public PigeonIMU gyro;
    public AHRS gyro;
    public ProfiledPIDController rotatePID;
    private Field2d field = new Field2d();


    public Swerve() {
        SmartDashboard.putData(field);
        limelight = new Limelight();
        // gyro = new PigeonIMU(Constants.Swerve.pigeonID);
        gyro  = new AHRS();
        
        zeroGyro();
        
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw());

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        
        rotatePID = new ProfiledPIDController(
                Constants.rotatePid_P,
                Constants.rotatePid_I,
                Constants.rotatePid_D, new TrapezoidProfile.Constraints(1, 3));

        rotatePID.setTolerance(Constants.rotatePid_Tol);

    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    
    // public double speed(){
    //     return 
    // }
    
    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
        
        
    }

    public double getVelocityMag() {
        return Math.sqrt(gyro.getVelocityX() * gyro.getVelocityX() + gyro.getVelocityY() * gyro.getVelocityY() + gyro.getVelocityZ() * gyro.getVelocityZ());
        
    }
    public double getVelocity(){
        return gyro.getVelocityY();
    }

    public double[] getVelocityCorrected(){
        double a = (Double.valueOf(gyro.getYaw())+180) * Math.PI / 180;
        double x = gyro.getVelocityX();
        double y = gyro.getVelocityY();
        return new double[] { x * Math.cos(a) - y * Math.sin(a), x * Math.sin(a) + y * Math.cos(a) };
    }
    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(pose, getYaw());
    }

    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public void zeroGyro(){
        gyro.zeroYaw();
    }

    public Rotation2d getYaw() {
      /*   double[] ypr = new double[3];
        gyro.;
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - ypr[0]) : Rotation2d.fromDegrees(ypr[0]); */

        if (gyro.isMagnetometerCalibrated()) {
            return Rotation2d.fromDegrees(gyro.getFusedHeading());
          }
           return Rotation2d.fromDegrees(360.0 - gyro.getYaw());
    }



  

    public double limelightOffset(){

        // float Kp = -0.1f;
double min_command = Constants.rotatePid_FF;


        double heading_error = limelight.tx();
        double steering_adjust = 0.0;
        if ((limelight.tx()+1) > 2)
        {
                steering_adjust = /* Kp*heading_error */ - min_command;
        }
        else if ((limelight.tx()+1) < 0)
        {
                steering_adjust =/*  Kp*heading_error + */ min_command;
        }

        if(limelight.tx() > ((limelight.getDistance()/12) * Constants.limelightDeadbandCoefficient) ||
           limelight.tx() <  -((limelight.getDistance()/12) * Constants.limelightDeadbandCoefficient)){
      return (rotatePID.calculate(limelight.tx(), 1) + steering_adjust);
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

    

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getStates());  
        field.setRobotPose(swerveOdometry.getPoseMeters());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }
}