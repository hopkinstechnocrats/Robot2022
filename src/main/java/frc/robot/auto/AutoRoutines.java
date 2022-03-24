// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Feed.FeedSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Launcher.LauncherSubsystem;
import frc.robot.subsystems.Limelight.LimelightSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.auto.FieldPositions;

/** Add your docs here. */
public class AutoRoutines {
        // variables
        private final DriveSubsystem m_robotDrive;
        private final FeedSubsystem m_feed;
        private final IntakeSubsystem m_intake;
        private final LimelightSubsystem m_limelight;
        private final LauncherSubsystem m_launcher;
        private final FieldPositions m_fieldPositions = new FieldPositions();

        public AutoRoutines(DriveSubsystem m_robotDrive, FeedSubsystem m_feed, IntakeSubsystem m_intake,
                        LimelightSubsystem m_limelight, LauncherSubsystem m_launcher) {
                // constructor
                this.m_robotDrive = m_robotDrive;
                this.m_feed = m_feed;
                this.m_intake = m_intake;
                this.m_limelight = m_limelight;
                this.m_launcher = m_launcher;
        }

        public SwerveControllerCommand DriveBetweenPoints(Translation2d startingPosition, Translation2d midPosition,
                        Translation2d endingPosition, DriveSubsystem m_robotDrive) {
                TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                                // Add kinematics to ensure max speed is actually obeyed
                                                .setKinematics(DriveConstants.kDriveKinematics);

                // An example trajectory to follow. All units in meters.
                Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                new Pose2d(startingPosition, new Rotation2d(0)),
                                // Pass through these two interior waypoints, making an 's' curve path
                                List.of(),
                                // End 3 meters straight ahead of where we started, facing forward
                                new Pose2d(endingPosition, new Rotation2d(0)), config);

                var thetaController = new ProfiledPIDController(AutoConstants.kPThetaController,
                                AutoConstants.kIThetaController, AutoConstants.kDThetaController,
                                AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(exampleTrajectory,
                                m_robotDrive::getPose, // Functional interface to feed supplier
                                DriveConstants.kDriveKinematics,

                                // Position controllers
                                new PIDController(AutoConstants.kPXController, 0, 0),
                                new PIDController(AutoConstants.kPYController, 0, 0), new ProfiledPIDController(0,0,0, AutoConstants.kThetaControllerConstraints),
                                m_robotDrive::setModuleStates, m_robotDrive);

                // Command autonomousLogCommand = new RunCommand(() -> {
                //         Logger.getInstance().recordOutput("DriveSubsystem/ThetaControllerGoalPosition",
                //                         thetaController.getGoal().position);
                //         Logger.getInstance().recordOutput("DriveSubsystem/ThetaControllerGoalVelocity",
                //                         thetaController.getGoal().velocity);
                //         Logger.getInstance().recordOutput("DriveSubsystem/ThetaControllerSetpointPosition",
                //                         thetaController.getSetpoint().position);
                //         Logger.getInstance().recordOutput("DriveSubsystem/ThetaControllerSetpointVelocity",
                //                         thetaController.getSetpoint().velocity);
                //         Logger.getInstance().recordOutput("DriveSubsystem/ThetaControllerPositionError",
                //                         thetaController.getPositionError());
                //         Logger.getInstance().recordOutput("DriveSubsystem/ThetaControllerVelocityError",
                //                         thetaController.getVelocityError());
                // });
                return swerveControllerCommand;
        }

        public SequentialCommandGroup TwoBallAutoRoutine(Pose2d zeroPose) {
                return new SequentialCommandGroup(new InstantCommand(() -> m_robotDrive.resetOdometry(zeroPose)),
                                new InstantCommand(m_intake::intakeIn), // actually out lol
                                new InstantCommand(m_intake::StartIntakeOut),
                                this.DriveBetweenPoints(zeroPose.getTranslation(),
                                               new Translation2d(-0.5,0), FieldPositions.R3, m_robotDrive).withTimeout(6),
                                // this.DriveBetweenPoints(new Translation2d(0, 0),
                                //                                 new Translation2d(1, 0), new Translation2d(2, 0), m_robotDrive),

                                // turn on launcher have enough time to speed up
                                new ParallelCommandGroup(new RunCommand(() -> m_launcher
                                                .spinFromDistance(Constants.LauncherConstants.heightOfHighHubReflectors
                                                                / (Math.tan(m_limelight.getVerticalAngle()))),
                                                m_launcher).withTimeout(15), new SequentialCommandGroup(
                                                                // aim the robot towards the hub
                                                                new RunCommand(() -> {
                                                                        m_robotDrive.drive(0, 0, -1 * m_limelight
                                                                                        .getRotationSpeed());
                                                                }, m_robotDrive).withTimeout(2.0), // this time is too
                                                                                                   // long...maybe
                                                                // turn on the feed
                                                                new RunCommand(() -> m_feed.spinFeed(-1), m_feed)
                                                                                .withTimeout(10),
                                                                // LAUNCH 2 balls (assuming recovery time)
                                                                // turn off feed
                                                                // turn off launcher
                                                                new InstantCommand(m_intake::intakeOut) // actually back
                                                                                                        // in lmao
                                // make sure off the tarmac

                                )));

        }
}
