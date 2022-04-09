package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import lib.iotemplates.ClosedLoopIO;
import lib.talonconfiguration.BaseTalonFXConfiguration;

public class ModuleDriveIO implements ClosedLoopIO {

    private final WPI_TalonFX driveMotor;
    private boolean inverted;
    double driveOutput;
    double velocitySetpointRadPerSec;

    private final PIDController m_drivePIDController = new PIDController(
            Constants.ModuleConstants.kPModuleDriveController, Constants.ModuleConstants.kIModuleDriveController,
            Constants.ModuleConstants.kDModuleDriveController);

    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0, 8.634, 0);

    String corners;

    public ModuleDriveIO(int motorPort, boolean inverted, String corners) {
        this.inverted = inverted;
        driveMotor = new WPI_TalonFX(motorPort, "GertrudeGreyser");
        // set status frame period of drive motor
        driveMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 20);
        driveMotor.configAllSettings(new BaseTalonFXConfiguration());
        driveMotor.setNeutralMode(NeutralMode.Brake);
        this.corners = corners;
    }

    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }

    public void updateInputs(ClosedLoopIOInputs inputs) {
        // inputs.positionRad = getPositionRad();
        inputs.velocityRadPerSec = getVelocityRadPerSecond();
        // inputs.appliedVolts = driveMotor.getMotorOutputVoltage();
        // inputs.statorCurrentAmps = new double [] {driveMotor.getStatorCurrent()};
        // inputs.supplyCurrentAmps = new double [] {driveMotor.getSupplyCurrent()};
        // inputs.tempCelcius = new double [] {driveMotor.getTemperature()};
        // inputs.velocitySetpointRadPerSec = velocitySetpointRadPerSec;
    }

    private double getPositionRad() {
        return Units.rotationsToRadians(
                driveMotor.getSelectedSensorPosition() / Constants.ModuleConstants.kDriveEncoderTicksPerRevolution);
    }

    private double getVelocityRadPerSecond() {
        return Units.rotationsToRadians(driveMotor.getSelectedSensorVelocity()
                / Constants.ModuleConstants.kDriveEncoderTicksPerRevolution * 10);
    }

    public void setVelocityRadPerSec(double desiredSpeedRadPerSecond) {
        velocitySetpointRadPerSec = desiredSpeedRadPerSecond;
        driveOutput = desiredSpeedRadPerSecond / 8.6
                + (
                inverted ?
                m_drivePIDController.calculate(getVelocityRadPerSecond(), desiredSpeedRadPerSecond)
                : -1*m_drivePIDController.calculate(getVelocityRadPerSecond(), desiredSpeedRadPerSecond));
        // Logger.getInstance().recordOutput(corners+" Error", getVelocityRadPerSecond()-desiredSpeedRadPerSecond);
        // Logger.getInstance().recordOutput(corners+" Setpoint", desiredSpeedRadPerSecond);
        // Logger.getInstance().recordOutput(corners+" Measurement", getVelocityRadPerSecond());

        driveMotor.setVoltage(driveOutput);
    }

}
