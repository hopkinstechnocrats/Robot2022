package lib.iotemplates;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import lib.util.TunableNumber;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class VelocityClosedLoopIOBase implements ClosedLoopIO{
    private String name;
    private TunableNumber kP;
    private TunableNumber kI;
    private TunableNumber kD;
    private TunableNumber kF;
    private PIDController feedback;
    private SimpleMotorFeedforward feedforward;
    private List<WPI_TalonSRX> motors;
    private final double kEncoderTicksPerRevolution;
    private double currentVelocity;
    private double setpoint;

    public VelocityClosedLoopIOBase(String name, int[] motorPort, double kP, double kI, double kD, double kF, double kEncoderTicksPerRevolution) {
        this.name = name;
        this.kEncoderTicksPerRevolution = kEncoderTicksPerRevolution;
        this.currentVelocity = 0;
        this.kP = new TunableNumber(name+"/kP", kP);
        this.kI = new TunableNumber(name+"/kI", kI);
        this.kD = new TunableNumber(name+"/kD", kD);
        this.kF = new TunableNumber(name+"/kF", kF);
        feedback = new PIDController(kP, kI, kD);
        feedforward = new SimpleMotorFeedforward(0, kF);
        motors = Arrays.stream(motorPort).mapToObj(WPI_TalonSRX::new).collect(Collectors.toList());
    }

    public void updateInputs(ClosedLoopIOInputs inputs) {
        inputs.appliedVolts = motors.get(0).getMotorOutputVoltage();
        for (int i = 0; i < motors.size(); i++) {
            inputs.supplyCurrentAmps[i] = motors.get(i).getSupplyCurrent();
            inputs.statorCurrentAmps[i] = motors.get(i).getStatorCurrent();
            inputs.tempCelcius[i] = motors.get(i).getTemperature();
            Faults faults = new Faults();
            motors.get(i).getFaults(faults);
            inputs.underVoltage[i] = faults.UnderVoltage;
            inputs.forwardLimitSwitch[i] = faults.ForwardLimitSwitch;
            inputs.reverseLimitSwitch[i] = faults.ReverseLimitSwitch;
            inputs.forwardSoftLimit[i] = faults.ForwardSoftLimit;
            inputs.reverseSoftLimit[i] = faults.ReverseSoftLimit;
            inputs.hardwareFailure[i] = faults.HardwareFailure;
            inputs.resetDuringEn[i] = faults.ResetDuringEn;
            inputs.sensorOverflow[i] = faults.SensorOverflow;
            inputs.sensorOutOfPhase[i] = faults.SensorOutOfPhase;
            inputs.hardwareESDReset[i] = faults.HardwareESDReset;
            inputs.remoteLossOfSignal[i] = faults.RemoteLossOfSignal;
            inputs.APIError[i] = faults.APIError;
            inputs.supplyOverV[i] = faults.SupplyOverV;
            inputs.supplyUnstable[i] = faults.SupplyUnstable;
        }

        System.out.println("KP SMD VALUE:"+kP.get());
        feedback.setP(kP.get());
        feedback.setI(kI.get());
        feedback.setD(kD.get());
        feedforward = new SimpleMotorFeedforward(0, kF.get());

        // Convert from raw sensor units to radians
        inputs.positionRad = Units.rotationsToRadians(motors.get(0).getSelectedSensorPosition() / kEncoderTicksPerRevolution);

        // Convert from raw units / 100ms to Rad/s
        inputs.velocityRadPerSec = 10 * Units.rotationsToRadians(motors.get(0).getSelectedSensorVelocity() / kEncoderTicksPerRevolution);
        currentVelocity = inputs.velocityRadPerSec;
        inputs.velocitySetpointRadPerSec = setpoint;
    }

    public void setVelocity(double velocityRadPerSec) {
        setpoint = velocityRadPerSec;
        Logger.getInstance().recordOutput("CURRENT VELOCITY", currentVelocity);
        Logger.getInstance().recordOutput("SETPOINT", velocityRadPerSec);
        double outputVoltage = feedforward.calculate(velocityRadPerSec) + feedback.calculate(currentVelocity, velocityRadPerSec);
        Logger.getInstance().recordOutput("FEEDBACK", feedback.calculate(currentVelocity, velocityRadPerSec));
        Logger.getInstance().recordOutput("FEEDFORWARD", feedforward.calculate(velocityRadPerSec));
        Logger.getInstance().recordOutput("OUTPUT", outputVoltage);
        for (WPI_TalonSRX motor : motors) {
            motor.setVoltage(outputVoltage);
        }
    }
}
