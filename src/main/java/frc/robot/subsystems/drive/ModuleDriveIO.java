package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import lib.iotemplates.ClosedLoopIO;
import lib.talonconfiguration.BaseTalonFXConfiguration;

public class ModuleDriveIO implements ClosedLoopIO {

    private final WPI_TalonFX driveMotor;
    private boolean inverted;

    private final PIDController m_drivePIDController =
            new PIDController(
                    Constants.ModuleConstants.kPModuleDriveController,
                    0,
                    0
            );

    public ModuleDriveIO(int motorPort, boolean inverted) {
        this.inverted = inverted;
        driveMotor = new WPI_TalonFX(motorPort);
        driveMotor.configAllSettings(new BaseTalonFXConfiguration());
        driveMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }

    public void updateInputs(ClosedLoopIOInputs inputs) {
        inputs.positionRad = getPositionRad();
        inputs.velocityRadPerSec = getVelocityRadPerSecond();
        inputs.appliedVolts = driveMotor.getMotorOutputVoltage();
        inputs.statorCurrentAmps = new double [] {driveMotor.getStatorCurrent()};
        inputs.supplyCurrentAmps = new double [] {driveMotor.getSupplyCurrent()};
        inputs.tempCelcius = new double [] {driveMotor.getTemperature()};
    }

    private double getPositionRad() {
        return Units.rotationsToRadians(driveMotor.getSelectedSensorPosition()/
                Constants.ModuleConstants.kDriveEncoderTicksPerRevolution);
    }

    private double getVelocityRadPerSecond() {
        return Units.rotationsToRadians(driveMotor.getSelectedSensorVelocity() /
                Constants.ModuleConstants.kDriveEncoderTicksPerRevolution * 10);
    }

    public void setVelocity(double desiredSpeedRadPerSecond) {
        final double driveOutput = inverted ?
                m_drivePIDController.calculate(getVelocityRadPerSecond(), desiredSpeedRadPerSecond)
                : -1*m_drivePIDController.calculate(getVelocityRadPerSecond(), desiredSpeedRadPerSecond);

        driveMotor.setVoltage(driveOutput);
    }

}
