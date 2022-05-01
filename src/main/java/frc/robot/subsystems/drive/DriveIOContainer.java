package frc.robot.subsystems.drive;

public class DriveIOContainer {
    public ModuleIOContainer frontLeft;
    public ModuleIOContainer frontRight;
    public ModuleIOContainer rearLeft;
    public ModuleIOContainer rearRight;

    public DriveIOContainer(ModuleIOContainer frontLeft, ModuleIOContainer frontRight, ModuleIOContainer rearLeft, ModuleIOContainer rearRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.rearLeft = rearLeft;
        this.rearRight = rearRight;
    }
}
