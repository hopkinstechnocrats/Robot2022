package frc.robot.subsystems.Climber;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lib.iotemplates.ClosedLoopIO.ClosedLoopIOInputs;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {

    public ClimberIO m_climberIO = new ClimberIO("Climb", 12, 7, 0.7, 0, 2048, .9, 1.25);
    DoubleSolenoid Clawssssssss = new DoubleSolenoid(PneumaticsModuleType.REVPH, 8, 9);
    ClosedLoopIOInputs inputs;
    int speed = 0;

    public ClimberSubsystem() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
        inputs = new ClosedLoopIOInputs(1);
    }

    public void spinClimber(double speed) {
        ClimberIO.motor.setVoltage(speed);
    }

    public void goTo(double Pos) {
        m_climberIO.setPosition(Pos);
    }

    public void clawsOut() {
        Clawssssssss.set(DoubleSolenoid.Value.kForward);
        System.out.println("Hooker Reppeled");
    }

    public void clawsIn() {
        Clawssssssss.set(DoubleSolenoid.Value.kReverse);
        System.out.println("Hooker Attracted");
    }

    public void periodic() {
        double startTime = Logger.getInstance().getRealTimestamp();
        Logger.getInstance().recordOutput("Climb/positionMeters", m_climberIO.getPosition());
        Logger.getInstance().recordOutput("Climb/positionErrorMeters", m_climberIO.feedback.getPositionError());
        Logger.getInstance().processInputs("Climb", inputs);
        m_climberIO.updateInputs(inputs);
        Logger.getInstance().recordOutput("Climb/goalMeters", m_climberIO.feedback.getGoal().position);
        Logger.getInstance().recordOutput("Climb/setpointMeters", m_climberIO.feedback.getSetpoint().position);
        double endTime = Logger.getInstance().getRealTimestamp();
        Logger.getInstance().recordOutput("ClimberCodeSec", endTime - startTime);
    }

    public void zeroClimberPosition() {
        m_climberIO.zeroPosition();
    }

    public void resetController() {
        m_climberIO.feedback.reset(m_climberIO.getPosition());
    }

    public double getPosition() {
        return m_climberIO.getPosition();
    }

    public void setPosition(double goal) {
        m_climberIO.setPosition(goal);
    }

}

