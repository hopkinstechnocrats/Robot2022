// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Climber.ClimberSubsystem;

public class AutoClimb extends CommandBase {
  ClimberSubsystem m_climber;
  /** Creates a new AutoCLimb. */
  public AutoClimb() {
    m_climber = new ClimberSubsystem();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new SequentialCommandGroup(
      // Onto Mid Rung
      new AutoExtendTelescope(1.4), 
      new AutoExtendTelescope(0), 
      new InstantCommand(() -> m_climber.clawsOut()), 
      new AutoExtendTelescope(1),
      // wait(),
      new waitUntilCommanand(() -> m_robotDrive.get
      // wait(),
      new AutoExtendTelescope(.66),
      new InstantCommand(() -> m_climber.clawsIn()),
      // wait()

      // Onto High Rung
      new AutoExtendTelescope(0), 
      new InstantCommand(() -> m_climber.clawsOut()), 
      new AutoExtendTelescope(1),
      // wait(),
      // wait(),
      new AutoExtendTelescope(1.4),
      new InstantCommand(() -> m_climber.clawsIn()),
      // wait()

      //Onto Traversal Rung
      new AutoExtendTelescope(0), 
      new InstantCommand(() -> m_climber.clawsOut()), 
      new AutoExtendTelescope(1),
      // wait(),
      // wait(),
      new AutoExtendTelescope(1.4),
      new InstantCommand(() -> m_climber.clawsIn())
      // wait()
      );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
