package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;
import lib.iotemplates.OpenLoopIO;

public class IntakeIOReal implements OpenLoopIO {
    /*
    Spark motor;

    public IntakeIOReal() {
        motor = new Spark(Constants.IntakeConstants.kPWMPort);
    }

    public void updateInputs(OpenLoopIOInputs inputs) {
        inputs.appliedVolts = motor.get() * RobotController.getBatteryVoltage();
    }

    public void setVoltage(double volts) {
        motor.set(volts / RobotController.getBatteryVoltage()); //convert to percent ouput for Spark controller
    }*/
}
