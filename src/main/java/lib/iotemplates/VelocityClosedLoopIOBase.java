package lib.iotemplates;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import lib.util.TunableNumber;

public class VelocityClosedLoopIOBase implements ClosedLoopIO{
    private String name;
    private TunableNumber kP;
    private TunableNumber kI;
    private TunableNumber kD;
    private TunableNumber kF;

    public VelocityClosedLoopIOBase(String name, int[] motorPort, double kP, double kI, double kD, double kF) {
        this.name = name;

    }

    public void updateInputs(ClosedLoopIOInputs inputs) {

    }

    public void setVelocity(double velocityRadPerSec) {

    }
}
