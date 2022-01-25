package lib.talonconfiguration;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public class BaseTalonFXConfiguration extends TalonFXConfiguration {
    public BaseTalonFXConfiguration() {
        super();
        supplyCurrLimit = new SupplyCurrentLimitConfiguration(true, 30, 30, 0.1);
        neutralDeadband = 0.05;
    }
}
