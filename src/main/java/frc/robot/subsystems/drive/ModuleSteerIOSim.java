package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import lib.iotemplates.ClosedLoopIO;

public class ModuleSteerIOSim implements ClosedLoopIO {

    Rotation2d positionSetpointRad;

    public ModuleSteerIOSim() {
        positionSetpointRad = new Rotation2d();
    }

    public void updateInputs(ClosedLoopIO.ClosedLoopIOInputs inputs) {
        // double startTime = Logger.getInstance().getRealTimestamp();
        // steerMotor.getMotorOutputVoltage();

        // double elapsedTime = Logger.getInstance().getRealTimestamp() - startTime;
        // Logger.getInstance().recordOutput("GetMotorOutputVoltageSec", elapsedTime);
        inputs.positionRad = getPosition().getRadians();
        // inputs.velocityRadPerSec = Units.degreesToRadians(encoder.getVelocity());
        // inputs.appliedVolts = steerMotor.getMotorOutputVoltage();
        // inputs.statorCurrentAmps = new double[] { steerMotor.getStatorCurrent() };
        // inputs.supplyCurrentAmps = new double[] { steerMotor.getSupplyCurrent() };
        // // inputs.tempCelcius = new double[] { steerMotor.getTemperature() };
        // inputs.positionSetpointRad = positionSetPointRad;

    }

    private Rotation2d getPosition() {
        return positionSetpointRad;
    }

    public void setPosition(Rotation2d positionRad) {
        positionSetpointRad = positionRad;
    }
}
