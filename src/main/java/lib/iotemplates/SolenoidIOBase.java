package lib.iotemplates;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class SolenoidIOBase implements SolenoidIO {
    private final DoubleSolenoid solenoid;

    public SolenoidIOBase(int fwdChannel, int revChannel) {
        solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, fwdChannel, revChannel);
    }

    @Override
    public void updateInputs(SolenoidIOInputs inputs) {
        inputs.revSolenoidDisabled = solenoid.isRevSolenoidDisabled();
        inputs.fwdSolenoidDisabled = solenoid.isFwdSolenoidDisabled();
        inputs.state = solenoid.get();
    }

    @Override
    public void set(DoubleSolenoid.Value state) {
        solenoid.set(state);
    }
}
