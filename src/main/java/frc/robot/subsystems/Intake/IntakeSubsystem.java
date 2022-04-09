package frc.robot.subsystems.Intake;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;
import lib.iotemplates.OpenLoopIO;

import lib.iotemplates.OpenLoopIOTalonSRXBase;
import lib.iotemplates.SolenoidIO;
import lib.iotemplates.SolenoidIOBase;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class IntakeSubsystem extends SubsystemBase {

    private final LEDSubsystem m_led;

    private final OpenLoopIOTalonSRXBase motorIO = new OpenLoopIOTalonSRXBase(Constants.IntakeConstants.kCANPort);
    private final WPI_VictorSPX kicker = new WPI_VictorSPX(1);
    private final OpenLoopIO.OpenLoopIOInputs motorInputs = new OpenLoopIO.OpenLoopIOInputs(1);

    private final SolenoidIOBase solenoidIO = new SolenoidIOBase(6, 7);
    private final SolenoidIO.SolenoidIOInputs solenoidInputs = new SolenoidIO.SolenoidIOInputs();

    double speed = 0;

    public IntakeSubsystem(LEDSubsystem led) {
        m_led = led;
    }

    public void spinIntake() {
        motorIO.setVoltage(-12 * speed);
        kicker.setVoltage(12 * speed);
    }

    public void RTrigger(XboxController m_controller) {
        if (m_controller.getRawAxis(10) == 1) {
            intakeIn();
        }
    }

    public void StartIntakeOut() {
        speed = -.8;
        spinIntake();
        m_led.intakeOn();
    }

    public void StartIntakeIn() {
        speed = .8;
        spinIntake();
    }

    public void EndIntake() {
        speed = 0;
        spinIntake();
        m_led.intakeOff();
    }

    public void intakeOut(){
        solenoidIO.set(DoubleSolenoid.Value.kForward);
    }

    public void intakeIn(){
        solenoidIO.set(DoubleSolenoid.Value.kReverse);
    }

    public void periodic() {
      motorIO.updateInputs(motorInputs);
       solenoidIO.updateInputs(solenoidInputs);
    }
}

