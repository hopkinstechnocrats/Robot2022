package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import lib.util.TunableNumber;


public class FixHeadingCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final ProfiledPIDController controller;
    private final Rotation2d desiredState;
    private final TunableNumber kP;
    private final TunableNumber kI;
    private final TunableNumber kD;


    public FixHeadingCommand(DriveSubsystem driveSubsystem, Rotation2d desiredState) {
        this.driveSubsystem = driveSubsystem;
        this.desiredState = desiredState;
        kP = new TunableNumber("FixHeadingkP", AutoConstants.kPThetaController);
        kI = new TunableNumber("FixHeadingkI", AutoConstants.kIThetaController);
        kD = new TunableNumber("FixHeadingkD", AutoConstants.kDThetaController);
        this.controller = new ProfiledPIDController(AutoConstants.kPThetaController, AutoConstants.kIThetaController, AutoConstants.kDThetaController, AutoConstants.kThetaControllerConstraints);
        controller.enableContinuousInput(-1*Math.PI, Math.PI);
            
        // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        System.out.println("FIXHEADINGCOMMAND");
        controller.setP(kP.get());
        controller.setI(kI.get());
        controller.setD(kD.get());
        var output = controller.calculate(MathUtil.angleModulus(driveSubsystem.getHeading().getRadians()), MathUtil.angleModulus(desiredState.getRadians()));
        driveSubsystem.driveNoDeadband(
                0,//-2*driverController.getLeftY(),
                0,//-2*driverController.getLeftX(),
                output);
        Logger.getInstance().recordOutput("DriveSubsystem/FixHeadingGoal", controller.getGoal().position);
        Logger.getInstance().recordOutput("DriveSubsystem/FixHeadingSetpoint", controller.getSetpoint().position);
        Logger.getInstance().recordOutput("DriveSubsystem/FixHeadingOutput", output);
        Logger.getInstance().recordOutput("DriveSubsystem/FixHeadingError", controller.getPositionError());
        


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
