// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lib.iotemplates;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Template hardware interface for a closed loop subsystem. */
public interface ClosedLoopIO {
    /** Contains all of the input data received from hardware. */
    public static class ClosedLoopIOInputs implements LoggableInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double[] supplyCurrentAmps = new double[] {};
        public double[] statorCurrentAmps = new double[] {};
        public double[] tempCelcius = new double[] {};
        public boolean[] underVoltage = new boolean[] {};
        public boolean[] forwardLimitSwitch = new boolean[] {};
        public boolean[] reverseLimitSwitch = new boolean[] {};
        public boolean[] forwardSoftLimit = new boolean[] {};
        public boolean[] reverseSoftLimit = new boolean[] {};
        public boolean[] hardwareFailure = new boolean[] {};
        public boolean[] resetDuringEn = new boolean[] {};
        public boolean[] sensorOverflow = new boolean[] {};
        public boolean[] sensorOutOfPhase = new boolean[] {};
        public boolean[] hardwareESDReset = new boolean[] {};
        public boolean[] remoteLossOfSignal = new boolean[] {};
        public boolean[] APIError = new boolean[] {};
        public boolean[] supplyOverV = new boolean[] {};
        public boolean[] supplyUnstable = new boolean[] {};

        public void toLog(LogTable table) {
            table.put("PositionRad", positionRad);
            table.put("VelocityRadPerSec", velocityRadPerSec);
            table.put("AppliedVolts", appliedVolts);
            table.put("SupplyCurrentAmps", supplyCurrentAmps);
            table.put("StatorCurrentAmps", statorCurrentAmps);
            table.put("TempCelcius", tempCelcius);
            table.put("UnderVoltage", underVoltage);
            table.put("ForwardLimitSwitch", forwardLimitSwitch);
            table.put("ReverseLimitSwitch", reverseLimitSwitch);
            table.put("ForwardSoftLimit", forwardSoftLimit);
            table.put("ReverseSoftLimit", reverseSoftLimit);
            table.put("HardwareFailure", hardwareFailure);
            table.put("ResetDuringEn", resetDuringEn);
            table.put("SensorOverflow", sensorOverflow);
            table.put("SensorOutOfPhase", sensorOutOfPhase);
            table.put("HardwareESDReset", hardwareESDReset);
            table.put("RemoteLossOfSignal", remoteLossOfSignal);
            table.put("APIError", APIError);
            table.put("SupplyOverV", supplyOverV);
            table.put("SupplyUnstable", supplyUnstable);
        }

        public void fromLog(LogTable table) {
            positionRad = table.getDouble("PositionRad", positionRad);
            velocityRadPerSec = table.getDouble("VelocityRadPerSec", velocityRadPerSec);
            appliedVolts = table.getDouble("AppliedVolts", appliedVolts);
            supplyCurrentAmps = table.getDoubleArray("StatorCurrentAmps", supplyCurrentAmps);
            statorCurrentAmps = table.getDoubleArray("StatorCurrentAmps", statorCurrentAmps);
            tempCelcius = table.getDoubleArray("TempCelcius", tempCelcius);
            underVoltage = table.getBooleanArray("UnderVoltage", underVoltage);
            forwardLimitSwitch = table.getBooleanArray("ForwardLimitSwitch", forwardLimitSwitch);
            reverseLimitSwitch = table.getBooleanArray("ReverseLimitSwitch", reverseLimitSwitch);
            forwardSoftLimit = table.getBooleanArray("ForwardSoftLimit", forwardSoftLimit);
            reverseSoftLimit = table.getBooleanArray("ReverseSoftLimit", reverseSoftLimit);
            hardwareFailure = table.getBooleanArray("HardwareFailure", hardwareFailure);
            resetDuringEn = table.getBooleanArray("ResetDuringEn", resetDuringEn);
            sensorOverflow = table.getBooleanArray("SensorOverflow", sensorOverflow);
            sensorOutOfPhase = table.getBooleanArray("SensorOutOfPhase", sensorOutOfPhase);
            hardwareESDReset = table.getBooleanArray("HardwareESDReset", hardwareESDReset);
            remoteLossOfSignal = table.getBooleanArray("RemoteLossOfSignal", remoteLossOfSignal);
            APIError = table.getBooleanArray("APIError", APIError);
            supplyOverV = table.getBooleanArray("SupplyOverV", supplyOverV);
            supplyUnstable = table.getBooleanArray("SupplyUnstable", supplyUnstable);
        }
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ClosedLoopIOInputs inputs) {
    }

    /** Run open loop at the specified voltage. */
    public default void setVoltage(double volts) {
    }

    /**
     * Run closed loop at the specified velocity.
     *
     * @param velocityRadPerSec Velocity setpoint.
     */
    public default void setVelocity(double velocityRadPerSec) {
    }

    public default void setPosition(double positionRad) {

    }

    /** Enable or disable brake mode. */
    public default void setBrakeMode(boolean enable) {
    }

    /** Set velocity PID constants. */
    public default void configurePID(double kp, double ki, double kd) {
    }

    /** Reset the encoder(s) to a known position. */
    public default void resetPosition(double positionRad) {
    }
}