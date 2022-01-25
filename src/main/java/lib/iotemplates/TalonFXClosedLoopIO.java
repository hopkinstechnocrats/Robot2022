package lib.iotemplates;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class TalonFXClosedLoopIO implements ClosedLoopIO {

    private final WPI_TalonFX motor;

    public TalonFXClosedLoopIO(int motorPort) {
        motor = new WPI_TalonFX(motorPort);
        motor.configAllSettings();
    }

    public void updateInputs(ClosedLoopIOInputs inputs) {
        inputs.positionRad = motor.getSelectedSensorPosition();
        inputs.velocityRadPerSec = motor.getSelectedSensorVelocity();
        inputs.appliedVolts = steerMotor.getMotorOutputVoltage();
        inputs.statorCurrentAmps = new double[] {steerMotor.getStatorCurrent()};
        inputs.supplyCurrentAmps = new double[] {steerMotor.getSupplyCurrent()};
        inputs.tempCelcius = new double[] {steerMotor.getTemperature()};
    }
}
