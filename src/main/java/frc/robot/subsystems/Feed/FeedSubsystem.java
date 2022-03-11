package frc.robot.subsystems.Feed;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.iotemplates.OpenLoopIO;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.littletonrobotics.junction.Logger;

public class FeedSubsystem extends SubsystemBase {

    private final TalonFX motor = new TalonFX(Constants.FeedConstants.kCANPort);


    public FeedSubsystem() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.
    }

    public void spinFeed(double speed) {
        motor.set(ControlMode.PercentOutput, speed);
    }

    public void periodic() {
       
    }
}

