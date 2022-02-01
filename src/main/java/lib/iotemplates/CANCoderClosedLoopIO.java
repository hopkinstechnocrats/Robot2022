package lib.iotemplates;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class CANCoderClosedLoopIO implements ClosedLoopIO {

    WPI_TalonFX steerMotor;
    CANCoder encoder;
    Rotation2d offset;

    public CANCoderClosedLoopIO(int motorPort, int encoderPort, boolean encoderReversed, double encoderOffset) {
        steerMotor = new WPI_TalonFX(motorPort);
        encoder = new CANCoder(encoderPort);
        offset = new Rotation2d(encoderOffset);
    }

    public void updateInputs(ClosedLoopIOInputs inputs) {
        inputs.positionRad = Rotation2d.fromDegrees(encoder.getAbsolutePosition()).minus(offset).getRadians();
        inputs.velocityRadPerSec = Units.degreesToRadians(encoder.getVelocity());
        inputs.appliedVolts = steerMotor.getMotorOutputVoltage();
        inputs.statorCurrentAmps = new double[] {steerMotor.getStatorCurrent()};
        inputs.supplyCurrentAmps = new double[] {steerMotor.getSupplyCurrent()};
        inputs.tempCelcius = new double[] {steerMotor.getTemperature()};
    }
}
