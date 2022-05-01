package lib.iotemplates;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface SolenoidIO {
    private static String convStateToString(DoubleSolenoid.Value state) {
        switch (state) {
            case kOff:
                return "Off";
            case kForward:
                return "Forward";
            case kReverse:
                return "Reverse";
        }
        return null;
    }

    private static DoubleSolenoid.Value convStringToState(String rawState) {
        switch (rawState) {
            case "Off":
                return DoubleSolenoid.Value.kOff;
            case "Forward":
                return DoubleSolenoid.Value.kForward;
            case "Reverse":
                return DoubleSolenoid.Value.kReverse;
        }
        return null;
    }

    void updateInputs(SolenoidIOInputs inputs);

    void set(DoubleSolenoid.Value state);

    class SolenoidIOInputs implements LoggableInputs {
        public boolean fwdSolenoidDisabled = false;
        public boolean revSolenoidDisabled = false;
        public DoubleSolenoid.Value state = DoubleSolenoid.Value.kOff;

        public void toLog(LogTable table) {
            table.put("FwdSolenoidDisabled", fwdSolenoidDisabled);
            table.put("RevSolenoidDisabled", revSolenoidDisabled);
            table.put("State", convStateToString(state));

        }

        public void fromLog(LogTable table) {
            fwdSolenoidDisabled = table.getBoolean("FwdSolenoidDisabled", fwdSolenoidDisabled);
            revSolenoidDisabled = table.getBoolean("RevSolenoidDisabled", revSolenoidDisabled);
            state = convStringToState(table.getString("State", convStateToString(state)));
        }

    }

}
