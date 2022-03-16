// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.FixHeadingCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Feed.FeedSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Launcher.LauncherSubsystem;
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
import lib.util.TunableNumber;
import org.littletonrobotics.junction.Logger;


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
//  String trajectoryJSON = "paths/output/GoGoGadgets.wpilib.json";
//  Trajectory trajectory = new Trajectory();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intake;
  private final ClimberSubsystem m_climber;
  private final FeedSubsystem m_feed;
  private final LauncherSubsystem m_launcher;
  public final Compressor phCompressor = new Compressor(PneumaticsModuleType.REVPH);
  public Pose2d zeroPose = new Pose2d(new Translation2d(0, 0), new Rotation2d());


  // private final SingleModuleTestFixture singleModuleTestFixture = new SingleModuleTestFixture();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
//    try {
//      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
//      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
//   } catch (IOException ex) {
//      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
//   }


    TunableNumber.setTuningMode(false);
    m_intake = new IntakeSubsystem();
    m_climber = new ClimberSubsystem();
    m_feed = new FeedSubsystem();
    m_launcher = new LauncherSubsystem();
      Solenoid obj = new Solenoid(PneumaticsModuleType.REVPH, 0);
      obj.set(true);
    phCompressor.enableAnalog(100, 120);
     // Configure the button bindings
    configureButtonBindings();
    m_intake.setDefaultCommand(
        new RunCommand( () -> 
            SmartDashboard.putNumber("Compressor Pressure", phCompressor.getPressure())
        , m_intake)
    );

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
                     3*m_driverController.getRightX()
                     ), m_robotDrive)); // use this to change from field oriented to non-field oriented

    m_intake.setDefaultCommand(
            new RunCommand(
                    m_intake::spinIntake
            , m_intake)
    );

    m_climber.setDefaultCommand(
        new RunCommand(
                () -> m_climber.spinClimber(0)
        , m_climber)
    );

    m_feed.setDefaultCommand(
      new RunCommand(
        () -> m_feed.spinFeed(0),
        m_feed)
    );

    TunableNumber launcherSpeed = new TunableNumber("launcher/launcherSpeedRPM", 0);

    m_launcher.setDefaultCommand(
      new RunCommand(
        () -> m_launcher.spinLauncher(launcherSpeed.get()),
        m_launcher)
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

      JoystickButton OAButton = new JoystickButton(m_operatorController, 1);
      JoystickButton OBButton = new JoystickButton(m_operatorController, 2);
      JoystickButton OXButton = new JoystickButton(m_operatorController, 3);
      JoystickButton OYButton = new JoystickButton(m_operatorController, 4);
      JoystickButton ORBumper = new JoystickButton(m_operatorController, 6);
      JoystickButton OLBumper = new JoystickButton(m_operatorController, 5);
      JoystickButton OBack = new JoystickButton(m_operatorController, 7);
      JoystickButton OStart = new JoystickButton(m_operatorController, 8);
      JoystickButton OLIn = new JoystickButton(m_operatorController, 9);
      JoystickButton ORIn = new JoystickButton(m_operatorController, 10);
      JoystickButton OXbox = new JoystickButton(m_operatorController, 13);
      // 
      POVButton DPadTop = new POVButton(m_driverController, 90);
      DPadTop.whenPressed(new FixHeadingCommand(m_robotDrive, Rotation2d.fromDegrees(90), m_driverController));
      POVButton DPadRight = new POVButton(m_driverController, 180);
      DPadRight.whenPressed(new FixHeadingCommand(m_robotDrive, Rotation2d.fromDegrees(180), m_driverController));
      POVButton DPadBottom = new POVButton(m_driverController, 270);
      DPadBottom.whenPressed(new FixHeadingCommand(m_robotDrive, Rotation2d.fromDegrees(270), m_driverController));
      POVButton DPadLeft = new POVButton(m_driverController, 0);
      DPadLeft.whenPressed(new FixHeadingCommand(m_robotDrive, Rotation2d.fromDegrees(0), m_driverController));

      POVButton ODPadTop = new POVButton(m_operatorController, 90);
      POVButton ODPadRight = new POVButton(m_operatorController, 180);
      POVButton ODPadBottom = new POVButton(m_operatorController, 270);
      POVButton ODPadLeft = new POVButton(m_operatorController, 0);

      AButton.whenPressed(new InstantCommand(m_robotDrive::zeroHeading));
      BButton.whenPressed(new InstantCommand(() -> m_robotDrive.resetOdometry(zeroPose)));
      YButton.whenPressed(new InstantCommand(m_robotDrive::fieldON));
      XButton.whenPressed(new InstantCommand(m_robotDrive::fieldOFF));

      OAButton.whenPressed(new InstantCommand(m_intake::intakeIn));
      OXButton.whenPressed(new InstantCommand(m_intake::intakeOut));
      OLBumper.toggleWhenActive(new StartEndCommand(m_intake::StartIntakeOut, m_intake::EndIntake));
      
      OBButton.whenPressed(new InstantCommand(m_climber::clawsOut));
      OYButton.whenPressed(new InstantCommand(m_climber::clawsIn));
      OStart.whenHeld(new RunCommand(() -> m_climber.spinClimber(3), m_climber));
      OBack.whenHeld(new RunCommand(() -> m_climber.spinClimber(-5), m_climber));
      OLIn.whenHeld(new RunCommand(() -> m_climber.spinClimber(12), m_climber));


      ORIn.whenHeld(new RunCommand(() -> m_launcher.spinFromDistance(m_robotDrive.getPose().getTranslation().getDistance(new Translation2d(0, 0))), m_launcher));
      ORBumper.whenHeld(new RunCommand(() -> m_feed.spinFeed(-1), m_feed));

      ODPadTop.whenHeld(new RunCommand(() -> m_launcher.spinLauncher(.75), m_launcher));
      ODPadLeft.whenHeld(new RunCommand(() -> m_launcher.spinLauncher(.1), m_launcher));
      ODPadRight.whenHeld(new RunCommand(() -> m_launcher.spinLauncher(.35), m_launcher));
      ODPadBottom.whenHeld(new RunCommand(() -> m_launcher.spinLauncher(.26), m_launcher));

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
            List.of(new Translation2d(0, 1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(0, 2, new Rotation2d(0)),
            config);

    var thetaController =
        new ProfiledPIDController(
            AutoConstants.kPThetaController, AutoConstants.kIThetaController, AutoConstants.kDThetaController, AutoConstants.kThetaControllerConstraints);
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

    Command autonomousLogCommand =
            new RunCommand(
                    () -> {
                        Logger.getInstance().recordOutput("DriveSubsystem/ThetaControllerGoalPosition", thetaController.getGoal().position);
                        Logger.getInstance().recordOutput("DriveSubsystem/ThetaControllerGoalVelocity", thetaController.getGoal().velocity);
                        Logger.getInstance().recordOutput("DriveSubsystem/ThetaControllerSetpointPosition", thetaController.getSetpoint().position);
                        Logger.getInstance().recordOutput("DriveSubsystem/ThetaControllerSetpointVelocity", thetaController.getSetpoint().velocity);
                        Logger.getInstance().recordOutput("DriveSubsystem/ThetaControllerPositionError", thetaController.getPositionError());
                        Logger.getInstance().recordOutput("DriveSubsystem/ThetaControllerVelocityError", thetaController.getVelocityError());
                    }
            );

    // Reset odometry to the starting pose of the trajectory.
//    m_robotDrive.resetOdometry(trajectory.getInitialPose());

    // new ParallelCommandGroup(
    //     new RunCommand(() -> m_launcher.spinLauncher(0.1)), m_launcher),
    //     new RunCommand(() -> m_feed.spinFeed(), m_feed)
    // ).andThen(swerveControllerCommand)
    // .andThen(() -> m_robotDrive.drive(0, 0, 0));
    
    // Run path following command, then stop at the end.
    m_robotDrive.fieldOFF();
    // return new SequentialCommandGroup(
    //   new InstantCommand(() -> m_robotDrive.resetOdometry(zeroPose)),
    //   new InstantCommand(() -> m_intake.intakeIn()),
    //   new RunCommand(() -> m_launcher.spinLauncher(0.18), m_launcher).withTimeout(2),
    //   new ParallelCommandGroup(
    //     new RunCommand(() -> m_launcher.spinLauncher(0.18),m_launcher).withTimeout(3),
    //     new RunCommand(() -> m_feed.spinFeed(-1), m_feed).withTimeout(3)
    //   ),
    //   new RunCommand(() -> m_robotDrive.drive(.7, 0, 0), m_robotDrive).withTimeout(8)
    // );

    return new SequentialCommandGroup(
      new InstantCommand(() -> m_robotDrive.resetOdometry(zeroPose)),
      new InstantCommand(m_intake::intakeIn),
      new RunCommand(() -> m_robotDrive.drive(1, 0, 0), m_robotDrive).withTimeout(1),
      new RunCommand(() -> m_robotDrive.drive(0, 0, 0), m_robotDrive).withTimeout(0.5),
      // new InstantCommand(() -> m_robotDrive.drive(0,0,0), m_robotDrive),
      new RunCommand(() -> m_launcher.spinLauncher(0.34), m_launcher).withTimeout(2),
      new ParallelCommandGroup(
        new RunCommand(() -> m_launcher.spinLauncher(0.34),m_launcher).withTimeout(7),
        new RunCommand(() -> m_feed.spinFeed(-1), m_feed).withTimeout(7)
      ),
      // new InstantCommand(m_intake::EndIntake),
      new InstantCommand(m_intake::intakeOut),
      new RunCommand(() -> m_robotDrive.drive(0.5, 0.1, 0), m_robotDrive).withTimeout(2)
      // new RunCommand(() -> m_robotDrive.drive(.7, 0, 0), m_robotDrive).withTimeout(8)
    );
    // )ParallelCommandGroup(
    //   swerveControllerCommand,

    //         // .deadlineWith(autonomousLogCommand)
    //         .andThen(() -> m_robotDrive.drive(0, 0, 0));
            
  }
}
