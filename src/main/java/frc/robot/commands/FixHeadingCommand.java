package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;


public class FixHeadingCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final PIDController controller;
    private final Rotation2d desiredState;
    private XboxController driverController;

    public FixHeadingCommand(DriveSubsystem driveSubsystem, Rotation2d desiredState, XboxController driverController ) {
        this.driveSubsystem = driveSubsystem;
        this.desiredState = desiredState;
        this.controller = new PIDController(3,0,0);
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
                -2*driverController.getLeftY(),
                -2*driverController.getLeftX(),
                output,
                true);
        Logger.getInstance().recordOutput("FixHeadingError", error.getRadians());
        Logger.getInstance().recordOutput("FixHeadingOutput", output);

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
