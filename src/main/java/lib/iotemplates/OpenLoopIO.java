// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lib.iotemplates;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Template hardware interface for an open loop subsystem.
 */
public interface OpenLoopIO {
    /**
     * Updates the set of loggable inputs.
     */
    default void updateInputs(OpenLoopIOInputs inputs) {
    }

    /**
     * Run open loop at the specified voltage.
     */
    default void setVoltage(double volts) {
    }

    /**
     * Enable or disable brake mode.
     */
    default void setBrakeMode(boolean enable) {
    }

    /**
     * Contains all of the input data received from hardware.
     */
    class OpenLoopIOInputs implements LoggableInputs {
        public double appliedVolts = 0.0;
        public double[] supplyCurrentAmps = new double[]{};
        public double[] statorCurrentAmps = new double[]{};
        public double[] tempCelcius = new double[]{};
        public boolean[] underVoltage = new boolean[]{};
        public boolean[] forwardLimitSwitch = new boolean[]{};
        public boolean[] reverseLimitSwitch = new boolean[]{};
        public boolean[] forwardSoftLimit = new boolean[]{};
        public boolean[] reverseSoftLimit = new boolean[]{};
        public boolean[] hardwareFailure = new boolean[]{};
        public boolean[] resetDuringEn = new boolean[]{};
        public boolean[] sensorOverflow = new boolean[]{};
        public boolean[] sensorOutOfPhase = new boolean[]{};
        public boolean[] hardwareESDReset = new boolean[]{};
        public boolean[] remoteLossOfSignal = new boolean[]{};
        public boolean[] APIError = new boolean[]{};
        public boolean[] supplyOverV = new boolean[]{};
        public boolean[] supplyUnstable = new boolean[]{};

        public OpenLoopIOInputs(int numMotors) {
            supplyCurrentAmps = new double[numMotors];
            statorCurrentAmps = new double[numMotors];
            tempCelcius = new double[numMotors];
            underVoltage = new boolean[numMotors];
            forwardLimitSwitch = new boolean[numMotors];
            reverseLimitSwitch = new boolean[numMotors];
            forwardSoftLimit = new boolean[numMotors];
            reverseSoftLimit = new boolean[numMotors];
            hardwareFailure = new boolean[numMotors];
            resetDuringEn = new boolean[numMotors];
            sensorOverflow = new boolean[numMotors];
            sensorOutOfPhase = new boolean[numMotors];
            hardwareESDReset = new boolean[numMotors];
            remoteLossOfSignal = new boolean[numMotors];
            APIError = new boolean[numMotors];
            supplyOverV = new boolean[numMotors];
            supplyUnstable = new boolean[numMotors];
        }

        public void toLog(LogTable table) {
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
}