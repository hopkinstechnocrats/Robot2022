// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoClimb extends SequentialCommandGroup {

  /** Creates a new AutoCLimb. */
  public AutoClimb(ClimberSubsystem m_climber, DriveSubsystem m_robotDrive) {

    addCommands(
      // Onto Mid Rung
      new AutoExtendTelescope(1.4), 
      new AutoExtendTelescope(0), 
      new InstantCommand(() -> m_climber.clawsOut()), 
      new WaitCommand(2),
      new AutoExtendTelescope(1),
      // wait(),
      new WaitUntilCommand(() -> m_robotDrive.getRoll() <= 60),
      new WaitUntilCommand(() -> m_robotDrive.getRoll() >= 60),
      // wait(),
      new AutoExtendTelescope(.66),
      new InstantCommand(() -> m_climber.clawsIn()),
      new WaitCommand(2),
      // wait()
      new WaitUntilCommand(() -> Math.abs(m_robotDrive.getRoll() - 60) < 5),

      // Onto High Rung
      new AutoExtendTelescope(0), 
      new InstantCommand(() -> m_climber.clawsOut()), 
      new WaitCommand(2),

      new AutoExtendTelescope(1),
      // wait(),
      new WaitUntilCommand(() -> m_robotDrive.getRoll() <= 60),
      new WaitUntilCommand(() -> m_robotDrive.getRoll() >= 60),
      // wait(),
      new AutoExtendTelescope(1.4),
      new InstantCommand(() -> m_climber.clawsIn()),
      new WaitCommand(2),

      // wait()
      new WaitUntilCommand(() -> Math.abs(m_robotDrive.getRoll() - 60) < 5),
      //Onto Traversal Rung
      new AutoExtendTelescope(0), 
      new InstantCommand(() -> m_climber.clawsOut()), 
      new WaitCommand(2),

      new AutoExtendTelescope(1),
      // wait(),
      new WaitUntilCommand(() -> m_robotDrive.getRoll() <= 60),
      new WaitUntilCommand(() -> m_robotDrive.getRoll() >= 60),
      // wait(),
      new AutoExtendTelescope(1.4),
      new InstantCommand(() -> m_climber.clawsIn())
      );
  }
}
