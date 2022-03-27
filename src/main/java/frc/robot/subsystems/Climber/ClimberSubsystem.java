package frc.robot.subsystems.Climber;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import lib.iotemplates.OpenLoopIO;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {

    private final WPI_TalonFX Motor = new WPI_TalonFX(12);
    DoubleSolenoid Clawssssssss = new DoubleSolenoid(PneumaticsModuleType.REVPH, 8, 9);
    ClimberIO m_climberIO = new ClimberIO("Climb", 12, 0, 0, 0, 2048, 1, 1.25);
    int speed = 0;

    public ClimberSubsystem() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
    }

    public void spinClimber(double speed) {
        Motor.setVoltage(speed);
    }
    public void goTo(double Pos){
        m_climberIO.setPosition(Pos);
    }
    public void clawsOut(){
        Clawssssssss.set(DoubleSolenoid.Value.kForward);
    }

    public void clawsIn(){
        Clawssssssss.set(DoubleSolenoid.Value.kReverse);
    }

    public void periodic() {
    }

}

