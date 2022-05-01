// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.FixHeadingCommand;
import frc.robot.subsystems.Feed.FeedSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Launcher.LauncherSubsystem;
import frc.robot.subsystems.Limelight.LimelightSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.List;

/**
 * Add your docs here.
 */
public class AutoRoutines {
    // variables
    private final DriveSubsystem m_robotDrive;
    private final FeedSubsystem m_feed;
    private final IntakeSubsystem m_intake;
    private final LimelightSubsystem m_limelight;
    private final LauncherSubsystem m_launcher;
    private final FieldPositions m_fieldPositions = new FieldPositions();
    private final SendableChooser<Command> chooser;

    public AutoRoutines(DriveSubsystem m_robotDrive, FeedSubsystem m_feed, IntakeSubsystem m_intake,
                        LimelightSubsystem m_limelight, LauncherSubsystem m_launcher) {
        // constructor
        this.m_robotDrive = m_robotDrive;
        this.m_feed = m_feed;
        this.m_intake = m_intake;
        this.m_limelight = m_limelight;
        this.m_launcher = m_launcher;
        chooser = new SendableChooser<>();
        chooser.setDefaultOption("3BallAutoFarSide", FiveBallAutoRoutine(FieldPositions.R3startingPosition));
        chooser.addOption("2BallAuto", twoBallAutoRoutine(FieldPositions.R3startingPosition));
    }

    public SendableChooser<Command> getAutoChooser() {
        return chooser;
    }

    public Command getAutoCommand() {
        return chooser.getSelected();
    }


    public Command DriveBetweenPoints(Translation2d startingPosition,
                                      Translation2d endingPosition, Rotation2d targetPosition, DriveSubsystem m_robotDrive) {
        TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics);
        var transform = endingPosition.minus(startingPosition);
        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(startingPosition, new Rotation2d(transform.getX(), transform.getY())),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(endingPosition, new Rotation2d(transform.getX(), transform.getY())), config);

        // var thetaController = new ProfiledPIDController(AutoConstants.kPThetaController,
        //                 AutoConstants.kIThetaController, AutoConstants.kDThetaController,
        //                 AutoConstants.kThetaControllerConstraints);
        var thetaController = new ProfiledPIDController(0, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        var xPIDController = new PIDController(AutoConstants.kPXController, 0, 0);
        var yPIDController = new PIDController(AutoConstants.kPYController, 0, 0);


        Command swerveControllerCommand = new SwerveControllerCommand(exampleTrajectory,
                m_robotDrive::getPose, // Functional interface to feed supplier
                DriveConstants.kDriveKinematics,
                // Position controllers
                xPIDController,
                yPIDController,
                thetaController,
                m_robotDrive::setModuleStates, m_robotDrive)
                .andThen(() -> m_robotDrive.drive(0, 0, 0));

        Command autonomousLogCommand = new RunCommand(() -> {
            Logger.getInstance().recordOutput("DriveSubsystem/ThetaControllerGoalPosition",
                    thetaController.getGoal().position);
            Logger.getInstance().recordOutput("DriveSubsystem/ThetaControllerGoalVelocity",
                    thetaController.getGoal().velocity);
            Logger.getInstance().recordOutput("DriveSubsystem/ThetaControllerSetpointPosition",
                    thetaController.getSetpoint().position);
            Logger.getInstance().recordOutput("DriveSubsystem/ThetaControllerSetpointVelocity",
                    thetaController.getSetpoint().velocity);
            Logger.getInstance().recordOutput("DriveSubsystem/ThetaControllerPositionError",
                    thetaController.getPositionError());
            Logger.getInstance().recordOutput("DriveSubsystem/ThetaControllerVelocityError",
                    thetaController.getVelocityError());
            Logger.getInstance().recordOutput("DriveSubsystem/XTargetPosition", xPIDController.getSetpoint());
            Logger.getInstance().recordOutput("DriveSubsystem/YTargetPosition", yPIDController.getSetpoint());
        });
        return new ParallelCommandGroup(swerveControllerCommand, autonomousLogCommand);
    }

    public Command drivePositiveX() {
        return new SequentialCommandGroup(new InstantCommand(() -> m_robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
                this.DriveBetweenPoints(new Translation2d(0, 0), new Translation2d(1, 0), new Rotation2d(0), m_robotDrive));
    }

    public Command drivePositiveY() {
        return new SequentialCommandGroup(new InstantCommand(() -> m_robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
                this.DriveBetweenPoints(new Translation2d(0, 0), new Translation2d(0, 1), new Rotation2d(0), m_robotDrive));
    }

    public SequentialCommandGroup TwoBallAutoRoutine(Pose2d zeroPose) {
        return new SequentialCommandGroup(new InstantCommand(() -> m_robotDrive.resetOdometry(zeroPose)),
                new InstantCommand(m_intake::intakeIn), // actually out lol
                new InstantCommand(m_intake::StartIntakeOut),
                this.DriveBetweenPoints(zeroPose.getTranslation(),
                                FieldPositions.R3, new Rotation2d(0), m_robotDrive)
                        .withTimeout(6),
                // this.DriveBetweenPoints(new Translation2d(0, 0),
                // new Translation2d(1, 0), new Translation2d(2, 0), m_robotDrive),
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

    // red three ball auto routine
    public SequentialCommandGroup ThreeBallAutoRoutine(Pose2d zeroPose) {

        return new SequentialCommandGroup(
                new InstantCommand(() -> m_robotDrive.resetOdometry(zeroPose)),
                new ParallelCommandGroup(
                        new RunCommand(() -> m_launcher.spinLauncher(3000), m_launcher).withTimeout(4),
                        // new RunCommand(() -> m_launcher
                        //                 .spinFromDistance(m_robotDrive.getPose().getTranslation().getNorm()),
                        //                 m_launcher).withTimeout(4), 
                        new RunCommand(() -> m_feed.spinFeed(-1), m_feed).withTimeout(4)
                ),
                new InstantCommand(m_intake::intakeIn),
                new ParallelCommandGroup(
                        this.DriveBetweenPoints(
                                zeroPose.getTranslation(),
                                FieldPositions.R1,
                                Rotation2d.fromDegrees(270),
                                m_robotDrive).withTimeout(5),
                        new StartEndCommand(() -> m_intake.StartIntakeOut(), () -> m_intake.EndIntake(), m_intake).withTimeout(5)
                ),
                new ParallelCommandGroup(
                        this.DriveBetweenPoints(
                                FieldPositions.R1,
                                FieldPositions.R2,
                                new Rotation2d(150),
                                m_robotDrive).withTimeout(5),
                        new StartEndCommand(() -> m_intake.StartIntakeOut(), () -> m_intake.EndIntake(), m_intake).withTimeout(5)

                ),
                // new RunCommand(() -> m_robotDrive.drive(0, 0, -1*m_limelight.getRotationSpeed()), m_robotDrive).withTimeout(2),
                new ParallelCommandGroup(
                        new RunCommand(() -> m_launcher.spinLauncher(3000), m_launcher).withTimeout(10),
                        // new RunCommand(() -> m_launcher
                        //                 .spinFromDistance(m_robotDrive.getPose().getTranslation().getNorm()),
                        //                 m_launcher).withTimeout(15), 
                        new SequentialCommandGroup(
                                new RunCommand(() -> m_feed.spinFeed(0), m_feed).withTimeout(2),
                                // new RunCommand(() -> {
                                //                                 m_robotDrive.drive(0, 0, -1 * m_limelight
                                //                                                 .getRotationSpeed());
                                //                         }, m_robotDrive).withTimeout(2.0), 
                                new RunCommand(() -> m_feed.spinFeed(-1), m_feed)
                                        .withTimeout(10),
                                new InstantCommand(m_intake::intakeOut)

                        )));


    }

    public SequentialCommandGroup FiveBallAutoRoutine(Pose2d zeroPose) {

        return new SequentialCommandGroup(
                new InstantCommand(() -> m_robotDrive.resetOdometry(zeroPose)),

                new InstantCommand(m_intake::intakeIn),
                new ParallelCommandGroup(
                        this.DriveBetweenPoints(
                                zeroPose.getTranslation(),
                                FieldPositions.R1,
                                Rotation2d.fromDegrees(250),
                                m_robotDrive).withTimeout(10),
                        new StartEndCommand(() -> m_intake.StartIntakeOut(), () -> m_intake.EndIntake(), m_intake).withTimeout(10),
                        new RunCommand(() -> m_launcher.spinLauncher(6100), m_launcher).withTimeout(10),
                        // new RunCommand(() -> m_launcher
                        //                 .spinFromDistance(m_robotDrive.getPose().getTranslation().getNorm()),
                        //                 m_launcher).withTimeout(4),
                        new RunCommand(() -> m_feed.spinFeed(-1), m_feed).withTimeout(10)
                ),
                new InstantCommand(() -> {
                    m_launcher.stopLauncher();
                    m_feed.spinFeed(0);
                }, m_launcher, m_feed),
                new ParallelCommandGroup(
                        this.DriveBetweenPoints(
                                FieldPositions.R1,
                                FieldPositions.R2,
                                new Rotation2d(150),
                                m_robotDrive).withTimeout(5),
                        new StartEndCommand(() -> m_intake.StartIntakeOut(), () -> m_intake.EndIntake(), m_intake).withTimeout(5)

                ),
                new FixHeadingCommand(m_robotDrive, new Rotation2d(-1.1)).withTimeout(3).andThen(new InstantCommand(() -> m_robotDrive.drive(0, 0, 0), m_robotDrive)),
                // new RunCommand(() -> m_robotDrive.drive(0, 0, -1*m_limelight.getRotationSpeed()), m_robotDrive).withTimeout(2),
                new ParallelCommandGroup(
                        new RunCommand(() -> m_launcher.spinLauncher(5500), m_launcher).withTimeout(10),
                        // new RunCommand(() -> m_launcher
                        //                 .spinFromDistance(m_robotDrive.getPose().getTranslation().getNorm()),
                        //                 m_launcher).withTimeout(15),
                        new StartEndCommand(() -> m_intake.StartIntakeOut(), () -> m_intake.EndIntake(), m_intake).withTimeout(5),
                        new SequentialCommandGroup(
                                // new RunCommand(() -> m_feed.spinFeed(0), m_feed).withTimeout(10),
                                // new RunCommand(() -> {
                                //                                 m_robotDrive.drive(0, 0, -1 * m_limelight
                                //                                                 .getRotationSpeed());
                                //                         }, m_robotDrive).withTimeout(2.0),
                                new RunCommand(() -> m_feed.spinFeed(-1), m_feed)
                                        .withTimeout(10)
                                // new InstantCommand(m_intake::intakeOut)
                        )
                ),
                new ParallelCommandGroup(
                        this.DriveBetweenPoints(
                                FieldPositions.R2,
                                FieldPositions.R7,
                                new Rotation2d(30),
                                m_robotDrive).withTimeout(5),
                        new StartEndCommand(() -> m_intake.StartIntakeOut(), () -> m_intake.EndIntake(), m_intake).withTimeout(5)

                ),
                new StartEndCommand(() -> m_intake.StartIntakeOut(), () -> m_intake.EndIntake(), m_intake).withTimeout(5)
        );


    }

    public SequentialCommandGroup twoBallAutoRoutine(Pose2d zeroPose) {

        return new SequentialCommandGroup(
                new InstantCommand(() -> m_robotDrive.resetOdometry(zeroPose)),

                new InstantCommand(m_intake::intakeIn),
                new ParallelCommandGroup(
                        new RunCommand(() -> m_robotDrive.drive(-1, 0, 0), m_robotDrive).withTimeout(1.5),
                        new StartEndCommand(() -> m_intake.StartIntakeOut(), () -> m_intake.EndIntake(), m_intake).withTimeout(1.5)
                ),
                new ParallelCommandGroup(
                        new RunCommand(() -> m_robotDrive.drive(0, 0, 0), m_robotDrive).withTimeout(10),
                        new StartEndCommand(() -> m_intake.StartIntakeOut(), () -> m_intake.EndIntake(), m_intake).withTimeout(10),
                        new RunCommand(() -> m_launcher.spinLauncher(6300), m_launcher).withTimeout(10),
                        // new RunCommand(() -> m_launcher
                        //                 .spinFromDistance(m_robotDrive.getPose().getTranslation().getNorm()),
                        //                 m_launcher).withTimeout(4),
                        new SequentialCommandGroup(
                                new WaitCommand(2),
                                new RunCommand(() -> m_feed.spinFeed(-1), m_feed).withTimeout(8))
                ),
                new InstantCommand(() -> {
                    m_launcher.stopLauncher();
                    m_feed.spinFeed(0);
                }, m_launcher, m_feed));


    }

    public Command driveStraightBetweenPoints(Translation2d startingPosition, Translation2d endingPosition, DriveSubsystem m_robotDrive) {
        TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics);
        var transform = endingPosition.minus(startingPosition);
        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(startingPosition, new Rotation2d(transform.getX(), transform.getY())),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(endingPosition, new Rotation2d(transform.getX(), transform.getY())), config);

        // var thetaController = new ProfiledPIDController(AutoConstants.kPThetaController,
        //                 AutoConstants.kIThetaController, AutoConstants.kDThetaController,
        //                 AutoConstants.kThetaControllerConstraints);
        var thetaController = new ProfiledPIDController(0, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        var xPIDController = new PIDController(AutoConstants.kPXController, 0, 0);
        var yPIDController = new PIDController(AutoConstants.kPYController, 0, 0);


        Command swerveControllerCommand = new SwerveControllerCommand(exampleTrajectory,
                m_robotDrive::getPose, // Functional interface to feed supplier
                DriveConstants.kDriveKinematics,
                // Position controllers
                xPIDController,
                yPIDController,
                thetaController,
                m_robotDrive::setModuleStates, m_robotDrive)
                .andThen(() -> m_robotDrive.drive(0, 0, 0));

        Command autonomousLogCommand = new RunCommand(() -> {
            Logger.getInstance().recordOutput("DriveSubsystem/ThetaControllerGoalPosition",
                    thetaController.getGoal().position);
            Logger.getInstance().recordOutput("DriveSubsystem/ThetaControllerGoalVelocity",
                    thetaController.getGoal().velocity);
            Logger.getInstance().recordOutput("DriveSubsystem/ThetaControllerSetpointPosition",
                    thetaController.getSetpoint().position);
            Logger.getInstance().recordOutput("DriveSubsystem/ThetaControllerSetpointVelocity",
                    thetaController.getSetpoint().velocity);
            Logger.getInstance().recordOutput("DriveSubsystem/ThetaControllerPositionError",
                    thetaController.getPositionError());
            Logger.getInstance().recordOutput("DriveSubsystem/ThetaControllerVelocityError",
                    thetaController.getVelocityError());
            Logger.getInstance().recordOutput("DriveSubsystem/XTargetPosition", xPIDController.getSetpoint());
            Logger.getInstance().recordOutput("DriveSubsystem/YTargetPosition", yPIDController.getSetpoint());
        });
        return new ParallelCommandGroup(swerveControllerCommand, autonomousLogCommand);
    }

//    public Command defensiveAutoRoutine() {
//        return new SequentialCommandGroup(
//                new InstantCommand(() -> m_RobotDrive.resetOdometry(FieldPositions .3 StartingPosition)),
//        new InstantCommand(m_intake::intakeIn),
//                new ParallelCommandGroup(
//                        this.DriveBetweenPoints(
//                                FieldPositions .3 StartingPosition,
//                FieldPositions.B3,
//                m_robotDrive).withTimeout(2)),
//        new ParallelCommandGroup(
//                this.DriveBetweenPoints(
//                        FieldPositions.B3,
//                        FieldPositions.R6,
//                        m_robotDrive).withTimeout(2)),
//                new ParallelCommandGroup(
//                        this.DriveBetweenPoints(
//                                FieldPositions.R6,
//                                FieldPositions.R5,
//                                m_robotDrive).withTimeout(2)),
//                                )

//    }
}
