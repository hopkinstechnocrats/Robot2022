// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;

public class AutoClimb extends SequentialCommandGroup {

    /**
     * Creates a new AutoCLimb.
     */
    public AutoClimb(ClimberSubsystem m_climber, DriveSubsystem m_robotDrive) {
        addCommands(
                // Onto Mid Rung
                // new AutoExtendTelescope(1.016, m_climber),
                new InstantCommand(() -> m_climber.clawsIn()),
                new AutoExtendTelescope(-0.02, m_climber, .9, 8),
                new InstantCommand(() -> m_climber.clawsOut()),
                new RunCommand(() -> m_climber.goTo(-0.02), m_climber).withTimeout(1),
                // new AutoExtendTelescope(0.05, m_climber),
                new RunCommand(() -> m_climber.spinClimber(3), m_climber).withTimeout(1.5),
                new InstantCommand(() -> m_climber.clawsIn()),
                new RunCommand(() -> m_climber.spinClimber(3), m_climber).withTimeout(0.5),
                new InstantCommand(() -> m_climber.clawsOut()),
                new AutoExtendTelescope(.65, m_climber, 1.25, 15)
                // wait(),
                // new WaitUntilCommand(() -> m_robotDrive.getRoll() <= 55),
                // new WaitUntilCommand(() -> m_robotDrive.getRoll() >= 40),
                // // // wait(),
                // new AutoExtendTelescope(1.016, m_climber, 1.25, 40),
                // new InstantCommand(() -> m_climber.clawsIn())
                // // wait()
                // new WaitUntilCommand(() -> Math.abs(m_robotDrive.getRoll() - 60) < 5),

                // // Onto High Rung
                // new AutoExtendTelescope(-0.03, m_climber, 0.7, 4),
                // new InstantCommand(() -> m_climber.clawsOut()),
                // new RunCommand(() -> m_climber.goTo(-0.02), m_climber).withTimeout(2),
                // new RunCommand(() -> m_climber.spinClimber(3), m_climber).withTimeout(1.5),
                // new InstantCommand(() -> m_climber.clawsIn()),
                // new RunCommand(() -> m_climber.spinClimber(3), m_climber).withTimeout(0.5),
                // new InstantCommand(() -> m_climber.clawsOut()),
                // new AutoExtendTelescope(.65, m_climber, 1.25, 15),
                // // wait(),
                // new WaitUntilCommand(() -> m_robotDrive.getRoll() <= 65),
                // new WaitUntilCommand(() -> m_robotDrive.getRoll() >= 40),
                // // // wait(),
                // new AutoExtendTelescope(1.016, m_climber, 1.25, 15),
                // new InstantCommand(() -> m_climber.clawsIn())
                // // wait()
                // new WaitUntilCommand(() -> Math.abs(m_robotDrive.getRoll() - 60) < 5),

                // // Onto High Rung
                // new WaitCommand(2),

                // new AutoExtendTelescope(.74, m_climber),
                // // wait(),
                // new WaitUntilCommand(() -> m_robotDrive.getRoll() <= 60),
                // new WaitUntilCommand(() -> m_robotDrive.getRoll() >= 60),
                // // wait(),
                // new AutoExtendTelescope(1.016, m_climber),
                // new InstantCommand(() -> m_climber.clawsIn()),
                // new WaitCommand(2),

                // // wait()
                // new WaitUntilCommand(() -> Math.abs(m_robotDrive.getRoll() - 60) < 5),
                // //Onto Traversal Rung
                // new AutoExtendTelescope(0, m_climber),
                // new InstantCommand(() -> m_climber.clawsOut()),
                // new WaitCommand(2),

                // new AutoExtendTelescope(.74, m_climber),
                // // wait(),
                // new WaitUntilCommand(() -> m_robotDrive.getRoll() <= 60),
                // new WaitUntilCommand(() -> m_robotDrive.getRoll() >= 60),
                // // wait(),
                // new AutoExtendTelescope(1.016), m_climber),
                // new InstantCommand(() -> m_climber.clawsIn())
        );

    }
}
