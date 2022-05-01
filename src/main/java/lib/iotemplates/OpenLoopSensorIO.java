// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lib.iotemplates;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Template hardware interface for an open loop subsystem.
 */
public interface OpenLoopSensorIO {
    /**
     * Updates the set of loggable inputs.
     */
    default void updateInputs(OpenLoopSensorIOInputs inputs) {
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
    class OpenLoopSensorIOInputs implements LoggableInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double appliedVolts = 0.0;
        public double[] currentAmps = new double[]{};
        public double[] tempCelcius = new double[]{};

        public void toLog(LogTable table) {
            table.put("PositionRad", positionRad);
            table.put("VelocityRadPerSec", velocityRadPerSec);
            table.put("AppliedVolts", appliedVolts);
            table.put("CurrentAmps", currentAmps);
            table.put("TempCelcius", tempCelcius);
        }

        public void fromLog(LogTable table) {
            positionRad = table.getDouble("PositionRad", positionRad);
            velocityRadPerSec = table.getDouble("VelocityRadPerSec", velocityRadPerSec);
            appliedVolts = table.getDouble("AppliedVolts", appliedVolts);
            currentAmps = table.getDoubleArray("CurrentAmps", currentAmps);
            tempCelcius = table.getDoubleArray("TempCelcius", tempCelcius);
        }
    }
}