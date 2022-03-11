package frc.robot.subsystems.Feed;


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

public class FeedSubsystem extends SubsystemBase {

    private final OpenLoopIOTalonSRXBase io = new OpenLoopIOTalonSRXBase(Constants.FeedConstants.kCANPort);
    private final OpenLoopIO.OpenLoopIOInputs inputs;

    public FeedSubsystem() {
        inputs = new OpenLoopIO.OpenLoopIOInputs();
    }

    public void spinFeed(double speed) {
        io.setVoltage(12 * speed);
    }

    public void periodic() {
       io.updateInputs(inputs);
       Logger.getInstance().processInputs("FeedMotor", inputs);
    }
}

