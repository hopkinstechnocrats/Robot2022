package frc.robot.subsystems.drive;

import lib.iotemplates.ClosedLoopIO;

public class ModuleIOContainer {
    public ClosedLoopIO drive;
    public ClosedLoopIO steer;

    public ModuleIOContainer(ClosedLoopIO drive, ClosedLoopIO steer) {
        this.drive = drive;
        this.steer = steer;
    }
}
