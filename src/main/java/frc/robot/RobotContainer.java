// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.FixHeadingCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.Intake.IntakeIOReal;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj.Filesystem;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.List;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.text.SimpleDateFormat;
import lib.Loggable;
import badlog.lib.BadLog;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  String trajectoryJSON = "paths/output/Unnamed.wpilib.json";
  Trajectory trajectory = new Trajectory();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intake;
  public Pose2d zeroPose = new Pose2d();
  // private final SingleModuleTestFixture singleModuleTestFixture = new SingleModuleTestFixture();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  public BadLog log;
  public File logFile;
  public BufferedWriter logFileWriter;
  private final List<Loggable> loggables;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
   } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
   }

    m_intake = new IntakeSubsystem(new IntakeIOReal());
    // Configure the button bindings
    configureButtonBindings();
    loggables = new ArrayList<Loggable>();
    // loggables.add(driv);

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () ->
                m_robotDrive.drive(
                    -2*m_driverController.getLeftY(),
                    -2*m_driverController.getLeftX(),
                     3*m_driverController.getRightX(),
                    true), m_robotDrive)); // use this to change from field oriented to non-field oriented

    m_intake.setDefaultCommand(
            new RunCommand(
                    () ->
                    {
                      m_intake.spinIntake();
                    }
            , m_intake)
    );

    // singleModuleTestFixture.setDefaultCommand(
    //         new RunCommand(
    //             () -> 
    //                 singleModuleTestFixture.setState(
    //                     m_driverController.getLeftY(), m_driverController.getRightY()),
    //             singleModuleTestFixture)
    // );
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
      JoystickButton AButton = new JoystickButton(m_driverController, 1);
      JoystickButton BButton = new JoystickButton(m_driverController, 2);
      JoystickButton XButton = new JoystickButton(m_driverController, 3);
      JoystickButton YButton = new JoystickButton(m_driverController, 4);
      // 
      POVButton DPadTop = new POVButton(m_driverController, 90);
      DPadTop.whenPressed(new FixHeadingCommand(m_robotDrive, Rotation2d.fromDegrees(90), m_driverController));
      POVButton DPadRight = new POVButton(m_driverController, 180);
      DPadRight.whenPressed(new FixHeadingCommand(m_robotDrive, Rotation2d.fromDegrees(180), m_driverController));
      POVButton DPadBottom = new POVButton(m_driverController, 270);
      DPadBottom.whenPressed(new FixHeadingCommand(m_robotDrive, Rotation2d.fromDegrees(270), m_driverController));
      POVButton DPadLeft = new POVButton(m_driverController, 0);
      DPadLeft.whenPressed(new FixHeadingCommand(m_robotDrive, Rotation2d.fromDegrees(0), m_driverController));

      AButton.whenPressed(new InstantCommand(() -> m_robotDrive.zeroHeading()));
      BButton.whenPressed(new InstantCommand(() -> m_robotDrive.resetOdometry(zeroPose)));
      XButton.toggleWhenPressed(new StartEndCommand(m_intake::startSpin,
              m_intake::endSpin,
              m_intake));
      YButton.toggleWhenPressed(new StartEndCommand(m_intake::startSpin,
              m_intake::endSpin,
              m_intake));
      // DPadTop.whenPressed(new InstantCommand(() -> .(90)));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, .5), new Translation2d(2, 0),
            new Translation2d(1, -.5)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(0, 0, new Rotation2d(-Math.PI/2)),
            config);

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, AutoConstants.kIThetaController, AutoConstants.kDThetaController, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setRotation,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }

  public void initializeLog() {
    String timeStamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(Calendar.getInstance().getTime());
    String filepath = "/home/lvuser/logs" + timeStamp + ".bag";


    File file = new File(filepath);
    try {
        //noinspection
        file.createNewFile();
        logFileWriter = new BufferedWriter(new FileWriter(file));
    } catch (IOException e) {
        DriverStation.reportError("File Creation error", e.getStackTrace());
    }

    log = BadLog.init(filepath);
    for (Loggable loggable : loggables) {
        loggable.logInit();
    }
  }
}
