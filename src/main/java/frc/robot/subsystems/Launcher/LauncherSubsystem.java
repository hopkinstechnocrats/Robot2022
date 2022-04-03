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
import lib.util.TunableNumber;

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
    TunableNumber spoid = new TunableNumber("Launcher/speed", 4000);

    public LauncherSubsystem() {
        interpolationTable = new LinearInterpolator();

        // interpolationTable.put(0, 0);
        interpolationTable.put(3.8, 6000);
        interpolationTable.put(5.2, 6450);
        //interpolationTable.put()
        
    }

    public void spinLauncher(double speedRPM) {
        io.setVelocity(Units.rotationsPerMinuteToRadiansPerSecond(speedRPM));
        SmartDashboard.putNumber("Launcher RPM", speedRPM);
    }


    public void spinLauncherTuning() {
        io.setVelocity(Units.rotationsPerMinuteToRadiansPerSecond(spoid.get()));
        SmartDashboard.putNumber("Launcher RPM", Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec));
    }

    public void stopLauncher() {
        io.setVoltage(0);
    }


    public void periodic() {
       io.updateInputs(inputs);
       Logger.getInstance().processInputs("launcher", inputs);
    }

    public void spinFromDistance(double distance) {
        spinLauncher(interpolationTable.get(distance));
    }
}

