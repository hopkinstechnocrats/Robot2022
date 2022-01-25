// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.Intake.IntakeIOReal;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.List;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
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
                    -1*m_driverController.getLeftY(),
                    -1*m_driverController.getLeftX(),
                    -6*m_driverController.getRightX(),
                    true), m_robotDrive)); // use this to change from field oriented to non-field oriented

    m_intake.setDefaultCommand(
            new RunCommand(
                    () ->
                    {
                      m_intake.spinIntake(0);
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
      // JoystickButton CButton = new JoystickButton(m_driverController, 3);
      // JoystickButton DButton = new JoystickButton(m_driverController, 4);
      BButton.whileHeld(new RunCommand(() -> {m_intake.spinIntake(12);}));
      // 
      POVButton DPadTop = new POVButton(m_driverController, 90);

      AButton.whenPressed(new InstantCommand(() -> m_robotDrive.zeroHeading()));
      BButton.whenPressed(new InstantCommand(() -> m_robotDrive.resetOdometry(zeroPose)));
      // CButton.whenPressed(new InstantCommand(() -> singleModuleTestFixture.setAngle(new Rotation2d(-1, 0))));
      // DButton.whenPressed(new InstantCommand(() -> singleModuleTestFixture.setAngle(new Rotation2d(0, -1))));
      
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
            List.of(new Translation2d(.4, 1), new Translation2d(.8, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(1.2, 0, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            m_robotDrive::getPose, // Functional interface to feed supplier
            DriveConstants.kDriveKinematics,

            // Position controllers
            new PIDController(AutoConstants.kPXController, 0, 0),
            new PIDController(AutoConstants.kPYController, 0, 0),
            thetaController,
            m_robotDrive::setModuleStates,
            m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }

  public void initializeLog() {
    String timeStamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(Calendar.getInstance().getTime());
    String filepath = "/home/lvuser/logs" + timeStamp + ".bag";


    File file = new File(filepath);
    try {
        //noinspection ResultOfMethodCallIgnored
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
