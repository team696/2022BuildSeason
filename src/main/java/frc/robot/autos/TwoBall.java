package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.AutoLimeLock;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeDelay;
import frc.robot.commands.LimelightLockSwerve;
import frc.robot.commands.SerializerCommand;
import frc.robot.subsystems.DIOSub;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Serializer;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TrajectoryTable;

import java.util.List;

import org.opencv.core.Mat;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TwoBall extends SequentialCommandGroup {
    public TwoBall(Swerve s_Swerve, 
                        Limelight limelight,
                        Shooter shooter,
                        Serializer serializer,
                        ShooterHood shooterHood,
                        TrajectoryTable trajectoryTable,
                        Intake intake,
                        Joystick controller){

        Command lockAndShoot = new ParallelCommandGroup(new AutoShoot(limelight, shooter, serializer, shooterHood, trajectoryTable).deadlineWith(
            new AutoLimeLock(s_Swerve, true , true )));
       
        TrajectoryConfig config =
            new TrajectoryConfig(
                    /* Constants.AutoConstants.kMaxSpeedMetersPerSecond */5,
                    /* Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared */5)
                .setKinematics(Constants.Swerve.swerveKinematics);
                config.setReversed(true);

                // config.setReversed(true );

        // An example trajectory to follow.  All units in meters.
        // Trajectory exampleTrajectory =
        //     TrajectoryGenerator.generateTrajectory(
        //         new Pose2d(0, 0.00, new Rotation2d(0)),
        //         List.of(new Translation2d(0.00, 0.00),
        //         new Translation2d(-1.1, 0)),
        //                  new Pose2d(-1.2, 0.00, new Rotation2d(0)),
        //         config);

        Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(-0.5, 0, new Rotation2d(0)),
            List.of(new Translation2d(-1.1, 0.001)),
                     new Pose2d(-2, -0.001, new Rotation2d(0)),
            config);
                // Trajectory exampleTrajectory = 

                // TrajectoryGenerator.generateTrajectory(
                //     List.of(
                //     new Pose2d(0, 0, new Rotation2d(0)),
                //     new Pose2d(-1, 0.0, new Rotation2d(0))),
                //      config);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand Step1 =
            new SwerveControllerCommand(
                exampleTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(/* Constants.AutoConstants.kPXController */0.8, 0, 0.004),
                new PIDController(/* Constants.AutoConstants.kPYController */0.8, 0, 0.004),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
        
            

        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
            Step1.deadlineWith(new IntakeCommand(intake, -0.4, true).alongWith(new SerializerCommand(serializer, 0.2, -0.6, shooter, 0 ))),
           lockAndShoot
         


        );
    }
}