// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber.ClimberIO;
import frc.robot.subsystems.Climber.ClimberSubsystem;

public class AutoExtendTelescope extends CommandBase {
  ClimberSubsystem m_climb;
  double Pos;
  double tolerance;
  /** Creates a new autoClimb. */
  public AutoExtendTelescope(double Pos) {
    this.Pos = Pos;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climb = new ClimberSubsystem();
    tolerance = .025;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_climb.goTo(Pos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return(Math.abs(ClimberIO.getPosition()-Pos)) <= (tolerance);
  }
}
