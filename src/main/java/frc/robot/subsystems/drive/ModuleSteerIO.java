package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import lib.iotemplates.ClosedLoopIO;
import lib.talonconfiguration.BaseTalonFXConfiguration;
import lib.util.TunableNumber;

public class ModuleSteerIO implements ClosedLoopIO {

    WPI_TalonFX steerMotor;
    CANCoder encoder;
    Rotation2d offset;
    double positionSetPointRad;
    TunableNumber kP = new TunableNumber("KPTuningController", 0);
    TunableNumber kI = new TunableNumber("KITuningController", 0);
    TunableNumber kD = new TunableNumber("KDTuningController", 0);

    private PIDController m_steeringPIDController = new PIDController(
            kP.get(), 
            kI.get(),
            kD.get());

    public ModuleSteerIO(int motorPort, int encoderPort, double encoderOffset) {
        steerMotor = new WPI_TalonFX(motorPort);
        steerMotor.configAllSettings(new BaseTalonFXConfiguration());
        steerMotor.setNeutralMode(NeutralMode.Brake);
        // set status frame period of steer motor
        steerMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
        m_steeringPIDController.enableContinuousInput(-Math.PI, Math.PI);
        encoder = new CANCoder(encoderPort);
        offset = new Rotation2d(encoderOffset);
    }

    public void updateInputs(ClosedLoopIOInputs inputs) {
        inputs.positionRad = getPosition().getRadians();
        inputs.velocityRadPerSec = Units.degreesToRadians(encoder.getVelocity());
        inputs.appliedVolts = steerMotor.getMotorOutputVoltage();
        inputs.statorCurrentAmps = new double[] { steerMotor.getStatorCurrent() };
        inputs.supplyCurrentAmps = new double[] { steerMotor.getSupplyCurrent() };
        inputs.tempCelcius = new double[] { steerMotor.getTemperature() };
        inputs.positionSetpointRad = positionSetPointRad;

        m_steeringPIDController.setP(kP.get());
        m_steeringPIDController.setI(kI.get());
        m_steeringPIDController.setD(kD.get());
    }

    private Rotation2d getPosition() {
        return Rotation2d.fromDegrees(encoder.getAbsolutePosition()).minus(offset);
    }

    public void setPosition(Rotation2d positionRad) {
        positionSetPointRad = positionRad.getRadians();
        final double turnOutput = m_steeringPIDController.calculate(
                getPosition().getRadians(),
                positionRad.getRadians()
        );

        steerMotor.setVoltage(turnOutput);
    }
}
