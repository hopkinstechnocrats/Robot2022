package frc.robot.subsystems.Launcher;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import lib.LinearInterpolator;
import lib.iotemplates.ClosedLoopIO;
import lib.iotemplates.OpenLoopIO;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import lib.iotemplates.VelocityClosedLoopIOBase;
import org.littletonrobotics.junction.Logger;

public class LauncherSubsystem extends SubsystemBase {

    private final VelocityClosedLoopIOBase io = new VelocityClosedLoopIOBase(
            "launcher",
            new int[] {Constants.LauncherConstants.kCANPort1, Constants.LauncherConstants.kCANPort2},
            Constants.LauncherConstants.kP,
            Constants.LauncherConstants.kI,
            Constants.LauncherConstants.kD,
            Constants.LauncherConstants.kF,
            Constants.LauncherConstants.kEncoderTicksPerRevolution
            );
    private final ClosedLoopIO.ClosedLoopIOInputs inputs = new ClosedLoopIO.ClosedLoopIOInputs(2);
    private LinearInterpolator interpolationTable;

    public LauncherSubsystem() {
        interpolationTable = new LinearInterpolator();

        // interpolationTable.put(0, 0);
        interpolationTable.put(2.34, 5200);
        interpolationTable.put(3.33, 6250);
        
    }

    public void spinLauncher(double speedRPM) {
        io.setVelocity(Units.rotationsPerMinuteToRadiansPerSecond(speedRPM));
    }


    public void periodic() {
       io.updateInputs(inputs);
       Logger.getInstance().processInputs("launcher", inputs);
    }

    public void spinFromDistance(double distance) {
        spinLauncher(interpolationTable.get(distance));
    }
}

