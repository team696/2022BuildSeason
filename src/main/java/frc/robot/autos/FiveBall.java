package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.AutoLimeLock;
import frc.robot.commands.AutoShoot;
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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FiveBall extends SequentialCommandGroup {
    public FiveBall(Swerve s_Swerve, 
                        Limelight limelight,
                        Shooter shooter,
                        Serializer serializer,
                        ShooterHood shooterHood,
                        TrajectoryTable trajectoryTable,
                        Intake intake,
                        Joystick controller){

        Command lockAndShoot = new ParallelCommandGroup(new AutoShoot(limelight, shooter, serializer, shooterHood, trajectoryTable).deadlineWith(
            new AutoLimeLock(s_Swerve, true , true )));
        Command lockAndShoot2 = new ParallelCommandGroup(new AutoShoot(limelight, shooter, serializer, shooterHood, trajectoryTable).deadlineWith(
                new AutoLimeLock(s_Swerve, true , true )));
                Command lockAndShoot3 = new ParallelCommandGroup(new AutoShoot(limelight, shooter, serializer, shooterHood, trajectoryTable).deadlineWith(
                    new AutoLimeLock(s_Swerve, true , true)));
        
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(-0.5, 0, new Rotation2d(0)),
                List.of(new Translation2d(-1.1, 0.001)),
                         new Pose2d(-1.4, -0.001, new Rotation2d(0)),
                config);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        var testThetaController = 
        new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
        testThetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand Step1 =
            new SwerveControllerCommand(
                exampleTrajectory,
                s_Swerve::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);
        
        SwerveControllerCommand Step2 =
                new SwerveControllerCommand(
                    TrajectoryGenerator.generateTrajectory(
                        new Pose2d(-1.5, -0.001, new Rotation2d(0)),
                        List.of(
                                new Translation2d(0, 1)),
                                new Pose2d(-0.1, 2.2, new Rotation2d(-180)),
                        config),
                    s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                    new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                    thetaController,
                    s_Swerve::setModuleStates,
                    s_Swerve);

        SwerveControllerCommand Step3 =
                new SwerveControllerCommand(
                    TrajectoryGenerator.generateTrajectory(
                        new Pose2d(-0.1, 2.2, new Rotation2d(-180)),
                        List.of(
                                new Translation2d(-.55, 6.4)),
                                new Pose2d(-0.4, 4, new Rotation2d(-180)),
                        config),
                    s_Swerve::getPose,
                    Constants.Swerve.swerveKinematics,
                    new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                    new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                    testThetaController,
                    s_Swerve::setModuleStates,
                    s_Swerve);
            
        SwerveControllerCommand Step4 =
        new SwerveControllerCommand(
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(-0.1, 2.2, new Rotation2d(-180)),
                List.of(
                        new Translation2d(-.55, 6.4)),
                        new Pose2d(-0.4, 5, new Rotation2d(0)),
                config),
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);
    
            
        

        addCommands(
        new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
            Step1.deadlineWith(new IntakeDelay(intake, -0.4, true).alongWith(new SerializerCommand(serializer, 0.2, -0.6))),
                lockAndShoot,
                    Step2.deadlineWith(new IntakeDelay(intake, -0.4, true).alongWith(new SerializerCommand(serializer, 0.2, -0.6))),
                        lockAndShoot2,
                            Step3.deadlineWith(new IntakeDelay(intake, -0.4, true).alongWith(new SerializerCommand(serializer, 0.2, -0.6))),
                                lockAndShoot3


        );
    }
}