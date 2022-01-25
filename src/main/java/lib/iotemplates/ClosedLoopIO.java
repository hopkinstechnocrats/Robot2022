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

        public void toLog(LogTable table) {
            table.put("PositionRad", positionRad);
            table.put("VelocityRadPerSec", velocityRadPerSec);
            table.put("AppliedVolts", appliedVolts);
            table.put("SupplyCurrentAmps", supplyCurrentAmps);
            table.put("StatorCurrentAmps", statorCurrentAmps);
            table.put("TempCelcius", tempCelcius);
        }

        public void fromLog(LogTable table) {
            positionRad = table.getDouble("PositionRad", positionRad);
            velocityRadPerSec = table.getDouble("VelocityRadPerSec", velocityRadPerSec);
            appliedVolts = table.getDouble("AppliedVolts", appliedVolts);
            supplyCurrentAmps = table.getDoubleArray("StatorCurrentAmps", supplyCurrentAmps);
            statorCurrentAmps = table.getDoubleArray("StatorCurrentAmps", statorCurrentAmps);
            tempCelcius = table.getDoubleArray("TempCelcius", tempCelcius);
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
     * @param ffVolts           Feed forward voltage from model.
     */
    public default void setVelocity(double velocityRadPerSec, double ffVolts) {
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