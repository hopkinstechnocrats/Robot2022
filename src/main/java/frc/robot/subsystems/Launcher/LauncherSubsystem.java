package frc.robot.subsystems.Launcher;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.iotemplates.OpenLoopIO;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.littletonrobotics.junction.Logger;

public class LauncherSubsystem extends SubsystemBase {

    private final WPI_TalonFX motor1 = new WPI_TalonFX(Constants.LauncherConstants.kCANPort1);
    private final WPI_TalonFX motor2 = new WPI_TalonFX(Constants.LauncherConstants.kCANPort2);


    public LauncherSubsystem() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
    }

    public void spinLauncher(double speed) {
        motor1.setVoltage(-1* 12* speed);
        motor2.setVoltage(-1* 12* speed);
    }


    public void periodic() {
       
    }
}

