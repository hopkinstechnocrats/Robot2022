package frc.robot.subsystems.Intake;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.iotemplates.OpenLoopIO;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.littletonrobotics.junction.Logger;

public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX motor = new TalonFX(Constants.IntakeConstants.kCANPort);

    double speed = 0;

    private final DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 6, 7);

    public IntakeSubsystem() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
    }

    public void spinIntake() {
        motor.set(ControlMode.PercentOutput, speed);
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
       
    }
}

