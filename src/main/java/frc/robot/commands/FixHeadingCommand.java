package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

import java.awt.*;


public class FixHeadingCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final PIDController controller;
    private final Rotation2d desiredState;
    private XboxController driverController;

    public FixHeadingCommand(DriveSubsystem driveSubsystem, Rotation2d desiredState, XboxController driverController ) {
        this.driveSubsystem = driveSubsystem;
        this.desiredState = desiredState;
        this.controller = new PIDController(0,0,0);
        this.driverController = driverController;

        // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        var error = desiredState.minus(driveSubsystem.getHeading());
        var output = controller.calculate(error.getRadians(),0);
        driveSubsystem.drive(
                -1*driverController.getLeftY(),
                -1*driverController.getLeftX(),
                output,
                true);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
