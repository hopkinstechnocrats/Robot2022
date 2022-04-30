package frc.robot.subsystems.Feed;


import edu.wpi.first.wpilibj.DigitalInput;
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

    private final OpenLoopIO io;
    private final OpenLoopIO.OpenLoopIOInputs inputs;
    private final DigitalInput bottomSensor;
    private final DigitalInput topSensor;

    public FeedSubsystem() {
        io = new OpenLoopIOTalonSRXBase(Constants.FeedConstants.kCANPort);
        inputs = new OpenLoopIO.OpenLoopIOInputs(1);
        bottomSensor = new DigitalInput(0);
        topSensor = new DigitalInput(1);
    }

    public FeedSubsystem(OpenLoopIO io) {
        this.io = io;
        inputs = new OpenLoopIO.OpenLoopIOInputs(1);
        bottomSensor = new DigitalInput(0);
        topSensor = new DigitalInput(1);
    }

    public void spinFeed(double speed) {
        io.setVoltage(12 * speed);
    }

    public void periodic() {
        double startTime = Logger.getInstance().getRealTimestamp();
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("FeedMotor", inputs);
        double endTime = Logger.getInstance().getRealTimestamp();
        Logger.getInstance().recordOutput("FeedCodeSec", endTime-startTime);
        Logger.getInstance().recordOutput("Feed/BottomSensorState", bottomSensor.get());
        Logger.getInstance().recordOutput("Feed/TopSensorState", topSensor.get());
    }

    public boolean getBottomSensor() {
        return bottomSensor.get();
    }
    public boolean getTopSensor() {
        return topSensor.get();
    }
}

