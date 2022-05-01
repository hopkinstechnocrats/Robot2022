package frc.robot.subsystems.drive;

import lib.iotemplates.ClosedLoopIO;

public class ModuleDriveIOSim implements ClosedLoopIO {
    double velocitySetpointRadPerSec;

    public ModuleDriveIOSim() {
        velocitySetpointRadPerSec = 0;
    }


    public void updateInputs(ClosedLoopIO.ClosedLoopIOInputs inputs) {
        // inputs.positionRad = getPositionRad();
        inputs.velocityRadPerSec = getVelocityRadPerSecond();
        // inputs.appliedVolts = driveMotor.getMotorOutputVoltage();
        // inputs.statorCurrentAmps = new double [] {driveMotor.getStatorCurrent()};
        // inputs.supplyCurrentAmps = new double [] {driveMotor.getSupplyCurrent()};
        // inputs.tempCelcius = new double [] {driveMotor.getTemperature()};
        // inputs.velocitySetpointRadPerSec = velocitySetpointRadPerSec;
    }

    private double getVelocityRadPerSecond() {
        return velocitySetpointRadPerSec;
    }

    public void setVelocityRadPerSec(double desiredSpeedRadPerSecond) {
        velocitySetpointRadPerSec = desiredSpeedRadPerSecond;
    }
}
