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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.auto.AutoRoutines;
import frc.robot.auto.FieldPositions;
import frc.robot.commands.AutoClimb;
import frc.robot.commands.AutoExtendTelescope;
import frc.robot.commands.FixHeadingCommand;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Feed.FeedSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Launcher.LauncherSubsystem;
import frc.robot.subsystems.Limelight.LimelightSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.util.Units;
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
  static DriveSubsystem m_robotDrive;
  private final LEDSubsystem m_led;
  private final IntakeSubsystem m_intake;
  final ClimberSubsystem m_climber;
  private final FeedSubsystem m_feed;
  private final LauncherSubsystem m_launcher;
  final LimelightSubsystem m_limelight;
  private final AutoRoutines myAutoRoutines;
  public final Compressor phCompressor = new Compressor(PneumaticsModuleType.REVPH);
  public Pose2d zeroPose = new Pose2d(new Translation2d(0, 0), new Rotation2d());


  // private final SingleModuleTestFixture singleModuleTestFixture = new SingleModuleTestFixture();

  // The driver's controller
  XboxController m_leftJoystick = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_rightJoystick = new XboxController(OIConstants.kOperatorControllerPort);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
//    try {
//      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
//      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
//   } catch (IOException ex) {
//      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
//   }


    TunableNumber.setTuningMode(true);
    m_led = new LEDSubsystem();
    m_robotDrive = new DriveSubsystem();
    m_intake = new IntakeSubsystem(m_led);
    m_climber = new ClimberSubsystem();
    m_feed = new FeedSubsystem();
    m_launcher = new LauncherSubsystem(m_led, m_leftJoystick, m_rightJoystick);
    m_limelight = new LimelightSubsystem();
    myAutoRoutines = new AutoRoutines(m_robotDrive, m_feed, m_intake, m_limelight, m_launcher); 
    SmartDashboard.putData(myAutoRoutines.getAutoChooser());
      Solenoid obj = new Solenoid(PneumaticsModuleType.REVPH, 0);
      obj.set(true);
    phCompressor.enableAnalog(100, 120);
     // Configure the button bindings
    configureButtonBindings();
    // m_intake.setDefaultCommand(
    //     new RunCommand( () -> 
    //         SmartDashboard.putNumber("Compressor Pressure", phCompressor.getPressure())
    //     , m_intake)
    // );

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    CommandBase driveDefaultCommand = new RunCommand(
      () ->
          m_robotDrive.drive(
              -2*m_leftJoystick.getRawAxis(2),
              -2*m_leftJoystick.getRawAxis(1),
               3*m_rightJoystick.getRawAxis(1)
               ), m_robotDrive);
    driveDefaultCommand.setName("DriveDefaultCommand");
    m_robotDrive.setDefaultCommand(driveDefaultCommand); // use this to change from field oriented to non-field oriented

    m_intake.setDefaultCommand(
            new RunCommand(m_intake::spinIntake, m_intake)
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
    CommandBase launcherDefaultCommand = new RunCommand(
      () ->  {
        // SmartDashboard.putNumber("Distance From Target", Constants.LauncherConstants.heightOfHighHubReflectors/(Math.tan(Units.degreesToRadians(m_limelight.getVerticalAngle()))));
        // m_limelight.ledsOff();
        // m_operatorController.setRumble(RumbleType.kLeftRumble, 0);
        // m_driverController.setRumble(RumbleType.kLeftRumble, 0);
        m_launcher.stopLauncher();
      }, //m_launcher.spinLauncher(launcherSpeed.get()); System.out.println("RUNNING LAUNCHER DEFAULT COMMAND");},
      m_launcher);
    
    launcherDefaultCommand.setName("LauncherDefaultCommand");

    m_launcher.setDefaultCommand(launcherDefaultCommand);
    
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
      JoystickButton RTrigger = new JoystickButton(m_rightJoystick, 1);
      JoystickButton R5Button = new JoystickButton(m_rightJoystick, 5);
      JoystickButton L5Button = new JoystickButton(m_leftJoystick, 5);
      JoystickButton L3Button = new JoystickButton(m_leftJoystick, 3);
      JoystickButton LTrigger = new JoystickButton(m_leftJoystick, 1);
      JoystickButton L6Button = new JoystickButton(m_leftJoystick, 6);
      JoystickButton L4Button = new JoystickButton(m_leftJoystick, 4);
      JoystickButton R6Button = new JoystickButton(m_rightJoystick, 6);
      JoystickButton R4Button = new JoystickButton(m_rightJoystick, 4);
      JoystickButton R3Button = new JoystickButton(m_rightJoystick, 3);
      JoystickButton R2Button = new JoystickButton(m_rightJoystick, 2);
 
      POVButton LDPadTop = new POVButton(m_leftJoystick, 0);
      LDPadTop.whenHeld(new RunCommand(() -> m_climber.spinClimber(8), m_climber));
      
      POVButton LDPadRight = new POVButton(m_leftJoystick, 90);
      
      Command autoClimb = new AutoClimb(m_climber, m_robotDrive)
              .beforeStarting(new InstantCommand(m_led::climbingOn))
              .andThen(new InstantCommand(m_led::climbingOff));
      
      LDPadRight.whenPressed(autoClimb);

      R2Button.whileHeld(new RunCommand(() -> m_robotDrive.drive(
        -4*m_leftJoystick.getRawAxis(2),
        -4*m_leftJoystick.getRawAxis(1),
         3*m_rightJoystick.getRawAxis(1)
         ), m_robotDrive));

      POVButton LDPadBottom = new POVButton(m_leftJoystick, 180);
      LDPadBottom.whenHeld(new RunCommand(() -> m_climber.spinClimber(-8), m_climber));
  
      POVButton LDPadLeft = new POVButton(m_leftJoystick, 270); 
      LDPadLeft.toggleWhenPressed(new StartEndCommand(m_climber::clawsOut, m_climber::clawsIn, m_climber));
      
      POVButton RDPadTop = new POVButton(m_rightJoystick, 0);
      POVButton RDPadRight = new POVButton(m_rightJoystick, 90);
      POVButton RDPadBottom = new POVButton(m_rightJoystick, 180);
      POVButton RDPadLeft = new POVButton(m_rightJoystick, 270);

      L6Button.whenPressed(() -> m_robotDrive.makeBackwards(true));
      L4Button.whenPressed(() -> m_robotDrive.makeBackwards(false));

      RTrigger.whenHeld(new SequentialCommandGroup(new RunCommand(()-> {m_robotDrive.driveNoDeadband(0, 0, m_limelight.getRotationSpeed());
        m_limelight.ledsOn();
        m_launcher.spinFromDistance(Constants.LauncherConstants.heightOfHighHubReflectors/(Math.tan(Units.degreesToRadians(m_limelight.getVerticalAngle()))));
      }, m_robotDrive).withInterrupt(() -> m_limelight.rotdeadzone()), 
        new RunCommand(() -> m_launcher.spinFromDistance(Constants.LauncherConstants.heightOfHighHubReflectors/(Math.tan(Units.degreesToRadians(m_limelight.getVerticalAngle())))), m_launcher).withInterrupt(() -> m_launcher.deadzoneIn()),
        new ParallelCommandGroup(new RunCommand(() -> m_launcher.spinFromDistance(Constants.LauncherConstants.heightOfHighHubReflectors/(Math.tan(Units.degreesToRadians(m_limelight.getVerticalAngle())))), m_launcher), 
        new RunCommand(() -> m_feed.spinFeed(-1), m_feed)).withInterrupt(() -> m_launcher.deadzoneOut()), 
        new RunCommand(() -> m_launcher.spinFromDistance(Constants.LauncherConstants.heightOfHighHubReflectors/(Math.tan(Units.degreesToRadians(m_limelight.getVerticalAngle())))), m_launcher).withInterrupt(() -> m_launcher.deadzoneIn()),
        new ParallelCommandGroup(new RunCommand(() -> m_launcher.spinFromDistance(Constants.LauncherConstants.heightOfHighHubReflectors/(Math.tan(Units.degreesToRadians(m_limelight.getVerticalAngle())))), m_launcher), 
        new RunCommand(() -> m_feed.spinFeed(-1), m_feed)).withInterrupt(() -> m_launcher.deadzoneOut())));

      R5Button.whenHeld(new RunCommand(() -> m_climber.setPosition(Units.inchesToMeters(62)), m_climber));

      L5Button.whenPressed(new InstantCommand(m_robotDrive::fieldON));
      L3Button.whenPressed(new InstantCommand(m_robotDrive::fieldOFF));

      LTrigger.whileHeld(new StartEndCommand(m_intake::StartIntakeOut,m_intake::EndIntake, m_intake));

      R4Button.whenHeld(new RunCommand(() -> m_feed.spinFeed(-1), m_feed));
      R6Button.whenHeld(new RunCommand(() -> m_feed.spinFeed(0.3), m_feed));
      
      R3Button.whileHeld(new RunCommand(() -> {
        m_launcher.spinFromDistance(Constants.LauncherConstants.heightOfHighHubReflectors/(Math.tan(Units.degreesToRadians(m_limelight.getVerticalAngle()))));
      }, m_launcher));

      RDPadTop.whenHeld(new RunCommand(() -> m_launcher.spinLauncher(6000), m_launcher));
      RDPadLeft.whenHeld(new RunCommand(() -> m_launcher.spinLauncher(3000), m_launcher));
      RDPadRight.whenHeld(new RunCommand(() -> m_launcher.spinLauncher(5000), m_launcher));
      RDPadBottom.whenHeld(new RunCommand(() -> m_launcher.spinLauncher(4000), m_launcher));
  }

  public void drive() {
     m_robotDrive.drive(
    -3*m_leftJoystick.getRawAxis(2),
    -3*m_leftJoystick.getRawAxis(1),
     3*m_rightJoystick.getRawAxis(1)
     );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return (new FixHeadingCommand(m_robotDrive, Rotation2d.fromDegrees(270)));
    return myAutoRoutines.getAutoCommand();
  }
}
