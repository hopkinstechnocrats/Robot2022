// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lib.iotemplates;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;


/**
 * Template hardware interface for a closed loop subsystem.
 */
public interface ClosedLoopIO {
    default void setVelocityRadPerSec(double v) {

    }

    default void setPosition(Rotation2d position) {

    }

    /**
     * Updates the set of loggable inputs.
     */
    default void updateInputs(ClosedLoopIOInputs inputs) {
    }

    /**
     * Run open loop at the specified voltage.
     */
    default void setVoltage(double volts) {
    }

    /**
     * Run closed loop at the specified velocity.
     *
     * @param velocityRadPerSec Velocity setpoint.
     */
    default void setVelocity(double velocityRadPerSec) {
    }

    default void setPosition(double positionRad) {

    }

    /**
     * Enable or disable brake mode.
     */
    default void setBrakeMode(boolean enable) {
    }

    /**
     * Set velocity PID constants.
     */
    default void configurePID(double kp, double ki, double kd) {
    }

    /**
     * Reset the encoder(s) to a known position.
     */
    default void resetPosition(double positionRad) {
    }

    /**
     * Contains all of the input data received from hardware.
     */
    class ClosedLoopIOInputs implements LoggableInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double velocitySetpointRadPerSec = 0;
        public double positionSetpointRad = 0;
        public double appliedVolts = 0.0;

        public ClosedLoopIOInputs(int numMotors) {

        }

        public void toLog(LogTable table) {
            table.put("PositionRad", positionRad);
            table.put("VelocityRadPerSec", velocityRadPerSec);
            table.put("SetpointRadPerSec", velocitySetpointRadPerSec);
            table.put("PositionSetpointRad", positionSetpointRad);
            table.put("AppliedVolts", appliedVolts);
        }

        public void fromLog(LogTable table) {
            positionRad = table.getDouble("PositionRad", positionRad);
            velocityRadPerSec = table.getDouble("VelocityRadPerSec", velocityRadPerSec);
            appliedVolts = table.getDouble("AppliedVolts", appliedVolts);
            velocitySetpointRadPerSec = table.getDouble("SetpointRadPerSec", velocitySetpointRadPerSec);
            positionSetpointRad = table.getDouble("PositionSetpointRad", positionSetpointRad);
        }
    }
}