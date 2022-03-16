package lib.iotemplates;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class OpenLoopIOTalonSRXBase implements OpenLoopIO {

    protected List<WPI_TalonSRX> motors;

    public OpenLoopIOTalonSRXBase(int... CANIDs) {
        motors = Arrays.stream(CANIDs).mapToObj(WPI_TalonSRX::new).collect(Collectors.toList());
    }

    public void updateInputs(OpenLoopIOInputs inputs) {
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
    }

    public void setVoltage(double voltage) {
        for (WPI_TalonSRX motor : motors) {
            motor.setVoltage(voltage);
        }
    }



}
