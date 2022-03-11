package frc.robot.subsystems.Intake;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.iotemplates.OpenLoopIO;

import lib.iotemplates.OpenLoopIOTalonSRXBase;
import lib.iotemplates.SolenoidIO;
import lib.iotemplates.SolenoidIOBase;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

    private final OpenLoopIOTalonSRXBase motorIO = new OpenLoopIOTalonSRXBase(Constants.IntakeConstants.kCANPort);
    private final OpenLoopIO.OpenLoopIOInputs motorInputs = new OpenLoopIO.OpenLoopIOInputs();

    private final SolenoidIOBase solenoidIO = new SolenoidIOBase(6, 7);
    private final SolenoidIO.SolenoidIOInputs solenoidInputs = new SolenoidIO.SolenoidIOInputs();

    double speed = 0;

    public void spinIntake() {
        motorIO.setVoltage(-12 * speed);
    }

    public void StartIntakeOut() {
        speed = -.8;
        spinIntake();
    }

    public void StartIntakeIn() {
        speed = .8;
        spinIntake();
    }

    public void EndIntake() {
        speed = 0;
        spinIntake();
    }

    public void intakeOut(){
        solenoidIO.set(DoubleSolenoid.Value.kForward);
    }

    public void intakeIn(){
        solenoidIO.set(DoubleSolenoid.Value.kReverse);
    }

    public void periodic() {
       motorIO.updateInputs(motorInputs);
       Logger.getInstance().processInputs("IntakeMotor", motorInputs);
       solenoidIO.updateInputs(solenoidInputs);
       Logger.getInstance().processInputs("IntakeSolenoid", solenoidInputs);
    }
}

