package lib.talonconfiguration;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

public class BaseTalonSRXConfiguration extends TalonSRXConfiguration {
    public BaseTalonSRXConfiguration() {
        super();
        neutralDeadband = 0.05;
    }
}
