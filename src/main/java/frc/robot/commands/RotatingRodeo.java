// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight.LimelightSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;


public class RotatingRodeo extends CommandBase {
    /**
     * Creates a new RotatingRodeo.
     */
    PIDController dipController = new PIDController(3, 0, 0);
    LimelightSubsystem limelight;

    public RotatingRodeo(DriveSubsystem robotdrive, LimelightSubsystem limelight) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(robotdrive, limelight);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // dipController.calculate(limelight.getX());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
