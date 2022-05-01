// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import lib.iotemplates.ClosedLoopIO;
import lib.util.TunableNumber;

/**
 * Add your docs here.
 */
public class ClimberIO implements ClosedLoopIO {

    static WPI_TalonFX motor;
    public ProfiledPIDController feedback;
    private final String name;
    private final TunableNumber kP;
    private final TunableNumber kI;
    private final TunableNumber kD;
    private double positionOffset;
    private final AnalogPotentiometer stringClimber;


    public ClimberIO(String name, int motorPort, double kP, double kI, double kD, double kEncoderTicksPerRevolution, double maxVelocity, double maxAcceleration) {
        this.name = name;
        stringClimber = new AnalogPotentiometer(0, 2.037, 0);
        this.kP = new TunableNumber(name + "/kP", kP);
        this.kI = new TunableNumber(name + "/kI", kI);
        this.kD = new TunableNumber(name + "/kD", kD);
        feedback = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        motor = new WPI_TalonFX(motorPort, "GertrudeGreyser");
        positionOffset = 0;
    }

    public void updateInputs(ClosedLoopIOInputs inputs) {
        // inputs.appliedVolts = motor.getMotorOutputVoltage();
        // inputs.supplyCurrentAmps[0] = motor.getSupplyCurrent();
        // inputs.statorCurrentAmps[0] = motor.getStatorCurrent();
        // // inputs.tempCelcius[0] = motor.getTemperature();
        // Faults faults = new Faults();
        // motor.getFaults(faults);
        // inputs.underVoltage[0] = faults.UnderVoltage;
        // inputs.forwardLimitSwitch[0] = faults.ForwardLimitSwitch;
        // inputs.reverseLimitSwitch[0] = faults.ReverseLimitSwitch;
        // inputs.forwardSoftLimit[0] = faults.ForwardSoftLimit;
        // inputs.reverseSoftLimit[0] = faults.ReverseSoftLimit;
        // inputs.hardwareFailure[0] = faults.HardwareFailure;
        // inputs.resetDuringEn[0] = faults.ResetDuringEn;
        // inputs.sensorOverflow[0] = faults.SensorOverflow;
        // inputs.sensorOutOfPhase[0] = faults.SensorOutOfPhase;
        // inputs.hardwareESDReset[0] = faults.HardwareESDReset;
        // inputs.remoteLossOfSignal[0] = faults.RemoteLossOfSignal;
        // inputs.APIError[0] = faults.APIError;
        // inputs.supplyOverV[0] = faults.SupplyOverV;
        // inputs.supplyUnstable[0] = faults.SupplyUnstable;
        feedback.setP(kP.get());
        feedback.setI(kI.get());
        feedback.setD(kD.get());
    }

    public double getPosition() {
        return stringClimber.get() - .365;
    }

    public void setPosition(double goal) {
        motor.set(feedback.calculate(getPosition(), goal));
    }

    public void setMaxSpeedAndAccel(double MaxSp, double MaxAcc) {
        feedback = new ProfiledPIDController(kP.get(), kI.get(), kD.get(), new TrapezoidProfile.Constraints(MaxSp, MaxAcc));
    }

    public void zeroPosition() {
        positionOffset = getPosition();
    }

}
        
