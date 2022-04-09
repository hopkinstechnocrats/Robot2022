// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package lib.iotemplates;


/** Template hardware interface for a closed loop subsystem. */
public interface ClosedLoopIO {
    /** Contains all of the input data received from hardware. */
    public static class ClosedLoopIOInputs {
        public double positionRad = 0.0;
        public double velocityRadPerSec = 0.0;
        public double velocitySetpointRadPerSec = 0;
        public double positionSetpointRad = 0;
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

        public ClosedLoopIOInputs(int numMotors) {
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