package frc.robot.subsystems.Intake;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.iotemplates.OpenLoopIO;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

    private final IntakeIOReal io;
    private final OpenLoopIO.OpenLoopIOInputs inputs = new OpenLoopIO.OpenLoopIOInputs();

    public IntakeSubsystem(IntakeIOReal io) {
        this.io = io;
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
    }

    public void spinIntake(double speed) {
        this.io.setVoltage(speed);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Intake", inputs);
    }
}

