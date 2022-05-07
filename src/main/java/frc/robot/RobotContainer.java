// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.OIConstants;
import frc.robot.auto.AutoRoutines;
import frc.robot.commands.AutoClimb;
import frc.robot.commands.AutoExtendTelescope;
import frc.robot.commands.AutomaticFeedCommand;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.auto.AutoRoutines;
import frc.robot.commands.AutoClimb;
import frc.robot.commands.AutomaticFeedCommand;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Feed.FeedSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Launcher.LauncherSubsystem;
import frc.robot.subsystems.Limelight.LimelightSubsystem;
import frc.robot.subsystems.drive.*;
import lib.util.TunableNumber;


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
    public final Compressor phCompressor = new Compressor(PneumaticsModuleType.REVPH);
    final ClimberSubsystem m_climber;
    final LimelightSubsystem m_limelight;
    private final LEDSubsystem m_led;
    private final IntakeSubsystem m_intake;
    final FeedSubsystem m_feed;
    private final LauncherSubsystem m_launcher;
    private final AutoRoutines myAutoRoutines;
    public Pose2d zeroPose = new Pose2d(new Translation2d(0, 0), new Rotation2d());
    public AutomaticFeedCommand autoFeedCommand;


    // private final SingleModuleTestFixture singleModuleTestFixture = new SingleModuleTestFixture();

    // The driver's controller
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(boolean isReal) {
//    try {
//      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
//      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
//   } catch (IOException ex) {
//      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
//   }
        TunableNumber.setTuningMode(true);
        m_robotDrive = new DriveSubsystem();
        m_led = new LEDSubsystem();
        m_intake = new IntakeSubsystem(m_led);
        m_climber = new ClimberSubsystem();
        m_feed = new FeedSubsystem();
        m_launcher = new LauncherSubsystem(m_led, m_driverController, m_operatorController);
        m_limelight = new LimelightSubsystem();
        myAutoRoutines = new AutoRoutines(m_robotDrive, m_feed, m_intake, m_limelight, m_launcher);
        SmartDashboard.putData(myAutoRoutines.getAutoChooser());
        Solenoid obj = new Solenoid(PneumaticsModuleType.REVPH, 0);
        obj.set(true);
        phCompressor.enableAnalog(100, 120);
        autoFeedCommand = new AutomaticFeedCommand(m_feed, () -> m_launcher.atSpeed() && m_limelight.isAimed() && (m_driverController.getAButton()));
        
      // OStart.whenHeld(new RunCommand(() -> m_climber.spinClimber(5), m_climber));
      // OBack.whenHeld(new RunCommand(() -> m_climber.spinClimber(-5), m_climber));
     
      
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
                                -3 * m_driverController.getLeftY(),
                                -3 * m_driverController.getLeftX(),
                                3 * m_driverController.getRightX()
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

        m_feed.setDefaultCommand(autoFeedCommand);

        TunableNumber launcherSpeed = new TunableNumber("launcher/launcherSpeedRPM", 0);
        CommandBase launcherDefaultCommand = new RunCommand(
                () -> {
                    // SmartDashboard.putNumber("Distance From Target", Constants.LauncherConstants.heightOfHighHubReflectors/(Math.tan(Units.degreesToRadians(m_limelight.getVerticalAngle()))));
                    // m_limelight.ledsOff();
                    // m_operatorController.setRumble(RumbleType.kLeftRumble, 0);
                    // m_driverController.setRumble(RumbleType.kLeftRumble, 0);
                    m_launcher.stopLauncher();
                    SmartDashboard.putBoolean("LauncherSpinning", false);
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
        JoystickButton AButton = new JoystickButton(m_driverController, 1);
        JoystickButton BButton = new JoystickButton(m_driverController, 2);
        JoystickButton XButton = new JoystickButton(m_driverController, 3);
        JoystickButton YButton = new JoystickButton(m_driverController, 4);
        JoystickButton LBumper = new JoystickButton(m_driverController, 5);
        JoystickButton RBumper = new JoystickButton(m_driverController, 6);
        JoystickButton Back = new JoystickButton(m_driverController, 7);
        JoystickButton Start = new JoystickButton(m_driverController, 8);

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

        // OLIn.whenPressed(new FixHeadingCommand(m_robotDrive, Rotation2d.fromDegrees(0)));
        // ORIn.whenPressed(new FixHeadingCommand(m_robotDrive, Rotation2d.fromDegrees(180)));
        //
        POVButton DPadTop = new POVButton(m_driverController, 0);
        DPadTop.whenHeld(new RunCommand(() -> m_climber.spinClimber(8), m_climber));
        // DPadTop.whenPressed(myAutoRoutines.drivePositiveY());
        POVButton DPadRight = new POVButton(m_driverController, 90);
        Command autoClimb = new AutoClimb(m_climber, m_robotDrive)
                .beforeStarting(new InstantCommand(m_led::climbingOn))
                .andThen(new InstantCommand(m_led::climbingOff));
        DPadRight.whenPressed(autoClimb);
        BButton.whenPressed(new InstantCommand(m_led::climbingOff, m_climber));
        // DPadRight.whenPressed(myAutoRoutines.drivePositiveX());
        POVButton DPadBottom = new POVButton(m_driverController, 180);
        DPadTop.whenHeld(new RunCommand(() -> m_climber.spinClimber(8), m_climber));
        //DPadBottom.whenHeld(new RunCommand(() -> m_launcher.spinLauncher(5500), m_launcher));
        //DPadBottom.whenPressed(new FixHeadingCommand(m_robo]\[][\]\tDrive, Rotation2d.fromDegrees(270)));
        DPadBottom.whenHeld(new RunCommand(() -> m_climber.spinClimber(-8), m_climber));
        POVButton DPadLeft = new POVButton(m_driverController, 270);
        DPadLeft.toggleWhenPressed(new StartEndCommand(m_climber::clawsOut, m_climber::clawsIn, m_climber));
        // DPadLeft.whenPressed(new FixHeadingCommand(m_robotDrive, Rotation2d.fromDegrees(0), m_driverController));

        POVButton ODPadTop = new POVButton(m_operatorController, 0);
        POVButton ODPadRight = new POVButton(m_operatorController, 90);
        POVButton ODPadBottom = new POVButton(m_operatorController, 180);
        POVButton ODPadLeft = new POVButton(m_operatorController, 270);

        Back.whenPressed(() -> m_robotDrive.makeBackwards(true));
        Start.whenPressed(() -> m_robotDrive.makeBackwards(false));
        // AButton.whenHeld(new RunCommand(() -> {m_robotDrive.driveNoDeadband(0, 0, m_limelight.getRotationSpeed());
        // m_limelight.ledsOn();}, m_robotDrive));
        RBumper.whenHeld(new RunCommand(() -> {
            m_robotDrive.driveNoDeadband(0, 0, m_limelight.getRotationSpeed());
        }));

        ORBumper.toggleWhenActive(new RunCommand(() -> {
            m_limelight.ledsOn();
            SmartDashboard.putBoolean("LauncherSpinning", true);
            m_launcher.spinFromDistance(Constants.LauncherConstants.heightOfHighHubReflectors / (Math.tan(Units.degreesToRadians(m_limelight.getVerticalAngle()))));
        }).beforeStarting(() -> {
            if (autoFeedCommand.getStateOfBall() == AutomaticFeedCommand.State.OneBallBottom) {
                autoFeedCommand.moveToTop();
            }}
        ));

        LBumper.whileHeld(new StartEndCommand(m_intake::StartIntakeOut, m_intake::EndIntake, m_intake));
        XButton.whenPressed(new InstantCommand(autoFeedCommand::resetEmpty));

        OXButton.whenHeld(new RunCommand(() -> m_feed.spinFeed(0.3), m_feed));
        OAButton.whenHeld(new RunCommand(() -> m_feed.spinFeed(-1), m_feed));

        OStart.whenHeld(new RunCommand(() -> m_climber.spinClimber(5), m_climber));
        OBack.whenHeld(new RunCommand(() -> m_climber.spinClimber(-5), m_climber));

        OLBumper.whileHeld(new RunCommand(() -> {
            m_launcher.spinFromDistance(Constants.LauncherConstants.heightOfHighHubReflectors / (Math.tan(Units.degreesToRadians(m_limelight.getVerticalAngle()))));
            // m_limelight.ledsOn();
        }, m_launcher));

      /*

      RBumper.whenHeld(new RunCommand(() -> m_climber.setPosition(Units.inchesToMeters(62)), m_climber));

      YButton.whenPressed(new InstantCommand(m_robotDrive::fieldON));
      XButton.whenPressed(new InstantCommand(m_robotDrive::fieldOFF));

      // LBumper.toggleWhenActive(new StartEndCommand(m_intake::StartIntakeOut, m_intake::EndIntake));
      // RBumper.whenPressed(new InstantCommand(m_intake::intakeIn));

      //OAButton.whenPressed(new InstantCommand(m_intake::intakeIn));
      //OXButton.whenPressed(new InstantCommand(m_intake::intakeOut));
     //OLBumper.toggleWhenActive(new StartEndCommand(m_intake::StartIntakeOut, m_intake::EndIntake));
      OYButton.whenHeld(new RunCommand(m_launcher::spinLauncherTuning, m_launcher));
      
      //OBButton.whenPressed(new InstantCommand(m_climber::clawsOut));
      //OYButton.whenPressed(new InstantCommand(m_climber::clawsIn));
      //OLIn.whenHeld(new RunCommand(() -> m_climber.spinClimber(12), m_climber));


      //ORIn.whenHeld(new RunCommand(() -> m_launcher.spinFromDistance(2.64/(Math.tan(m_limelight.getVerticalAngle()))), m_launcher));
      //ORBumper.whenHeld(new RunCommand(() -> m_feed.spinFeed(-1), m_feed));
      

      ODPadTop.whenHeld(new RunCommand(() -> m_launcher.spinLauncher(6000), m_launcher));
      ODPadLeft.whenHeld(new RunCommand(() -> m_launcher.spinLauncher(3000), m_launcher));
      ODPadRight.whenHeld(new RunCommand(() -> m_launcher.spinLauncher(5000), m_launcher));
      ODPadBottom.whenHeld(new RunCommand(() -> m_launcher.spinLauncher(4000), m_launcher));

      // Pull intake in on right trigger press
      // Trigger LTrigger = new Trigger(() -> m_driverController.getRawAxis(2) >= .7);

      // LTrigger.whenActive(new InstantCommand(m_intake::intakeOut));
      // DPadTop.whenPressed(new InstantCommand(() -> .(90))); */


        // Right trigger used to slow down the robot to 1.5 m/s

        Trigger RTrigger = new Trigger(() -> m_driverController.getRawAxis(3) >= .5);

        RTrigger.whenActive(new RunCommand(
                () ->
                        m_robotDrive.drive(
                                -1.5 * m_driverController.getLeftY(),
                                -1.5 * m_driverController.getLeftX(),
                                3 * m_driverController.getRightX()
                        ), m_robotDrive)
        );

      RTrigger.whenActive(new RunCommand(
        () ->
            m_robotDrive.drive(
                -1.5*m_driverController.getLeftY(),
                -1.5*m_driverController.getLeftX(),
                 3*m_driverController.getRightX()
                 ), m_robotDrive)
                 );
      // 1.2 and 0.9 are made up values
      // Extending the clibmer to right below the mid rung and right above the mid rung
      YButton.whenPressed(new AutoExtendTelescope(1.2, m_climber, 1.25, 15));
      OStart.whenPressed(new AutoExtendTelescope(0.9, m_climber, 1.25, 15));

      
      
  }

    

    public void drive() {
        m_robotDrive.drive(
                -3 * m_driverController.getLeftY(),
                -3 * m_driverController.getLeftX(),
                3 * m_driverController.getRightX()
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
