package frc.robot.subsystems.Intake;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.iotemplates.OpenLoopIO;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import lib.iotemplates.OpenLoopIOTalonSRXBase;
import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

    private final OpenLoopIOTalonSRXBase motorIO = new OpenLoopIOTalonSRXBase(Constants.IntakeConstants.kCANPort);
    private final OpenLoopIO.OpenLoopIOInputs motorInputs = new OpenLoopIO.OpenLoopIOInputs();

    double speed = 0;

    private final DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6, 7);

    public IntakeSubsystem() {

    }

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
        intakeSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void intakeIn(){
        intakeSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void periodic() {
       motorIO.updateInputs(motorInputs);
       Logger.getInstance().processInputs("IntakeMotor", motorInputs);
    }
}

