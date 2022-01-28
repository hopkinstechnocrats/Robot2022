// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lib.iotemplates;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Template hardware interface for an open loop subsystem. */
public interface OpenLoopIO {
    /** Contains all of the input data received from hardware. */
    public static class OpenLoopIOInputs implements LoggableInputs {
        public double appliedVolts = 0.0;

        public void toLog(LogTable table) {
            table.put("AppliedVolts", appliedVolts);
        }

        public void fromLog(LogTable table) {
            appliedVolts = table.getDouble("AppliedVolts", appliedVolts);
        }
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(OpenLoopIOInputs inputs) {
    }

    /** Run open loop at the specified voltage. */
    public default void setVoltage(double volts) {
    }

    /** Enable or disable brake mode. */
    public default void setBrakeMode(boolean enable) {
    }
}