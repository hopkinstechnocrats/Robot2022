package frc.robot.subsystems.Launcher;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;
import lib.LinearInterpolator;
import lib.iotemplates.ClosedLoopIO;
import lib.iotemplates.VelocityClosedLoopIOTalon;
import lib.util.TunableNumber;
import org.littletonrobotics.junction.Logger;

public class LauncherSubsystem extends SubsystemBase {

    private final ClosedLoopIO io;
    private final ClosedLoopIO.ClosedLoopIOInputs inputs = new ClosedLoopIO.ClosedLoopIOInputs(2);
    private final LinearInterpolator interpolationTable;
    private final LEDSubsystem m_led;
    XboxController m_leftController;
    XboxController m_rightController;
    TunableNumber spoid = new TunableNumber("Launcher/speed", 4000);
    TunableNumber scalingFactor = new TunableNumber("Launcher/ScalingFactor", 1);

    public LauncherSubsystem(LEDSubsystem led, XboxController m_lefController, XboxController m_rightController) {
        io = new VelocityClosedLoopIOTalon(
                "launcher",
                new int[]{Constants.LauncherConstants.kCANPort1, Constants.LauncherConstants.kCANPort2},
                Constants.LauncherConstants.kP,
                Constants.LauncherConstants.kI,
                Constants.LauncherConstants.kD,
                Constants.LauncherConstants.kF,
                Constants.LauncherConstants.kEncoderTicksPerRevolution
        );
        interpolationTable = new LinearInterpolator();

        // interpolationTable.put(0, 0);
        interpolationTable.put(3.08, 6000);
        interpolationTable.put(5.69, 6904);
        interpolationTable.put(7.80187249, 7565);
        m_led = led;
        this.m_leftController = m_lefController;
        this.m_rightController = m_rightController;
        //interpolationTable.put()

    }

    public LauncherSubsystem(LEDSubsystem led, XboxController m_leftController, XboxController m_rightController, ClosedLoopIO io) {
        this.io = io;
        interpolationTable = new LinearInterpolator();

        // interpolationTable.put(0, 0);
        interpolationTable.put(3.08, 6000);
        interpolationTable.put(5.69, 6904);
        interpolationTable.put(7.80187249, 7565);
        m_led = led;
        this.m_leftController = m_leftController;
        this.m_rightController = m_rightController;
        //interpolationTable.put()

    }

    public void spinLauncher(double speedRPM) {
        io.setVelocity(scalingFactor.get() * Units.rotationsPerMinuteToRadiansPerSecond(speedRPM));
        SmartDashboard.putNumber("Launcher RPM", speedRPM * scalingFactor.get());
        m_led.launchingOn();
        if (Units.radiansPerSecondToRotationsPerMinute(Math.abs(inputs.velocityRadPerSec - inputs.velocitySetpointRadPerSec)) < 300) {
            m_led.onTargetOn();
        } else {
            m_led.onTargetOff();
        }

        // if (Math.abs(inputs.velocityRadPerSec - inputs.velocitySetpointRadPerSec)<20) {
        //     m_operator.setRumble(RumbleType.kLeftRumble, 1);
        // } else {
        //     m_operator.setRumble(RumbleType.kLeftRumble, 0);
        // }
    }

    public Boolean atSpeed() {
        return Math.abs(inputs.velocityRadPerSec - inputs.velocitySetpointRadPerSec) < 40 && inputs.velocitySetpointRadPerSec != 0;
    }


    public void spinLauncherTuning() {
        io.setVelocity(Units.rotationsPerMinuteToRadiansPerSecond(spoid.get()));

    }

    public void stopLauncher() {
        io.setVoltage(0);
        m_led.launchingOff();
        m_led.onTargetOff();
    }


    public void periodic() {
        double startTime = Logger.getInstance().getRealTimestamp();
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("launcher", inputs);
        SmartDashboard.putNumber("Launcher RPM", Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec));
        SmartDashboard.putBoolean("At Speed", Math.abs(inputs.velocityRadPerSec - inputs.velocitySetpointRadPerSec) < 20);
        SmartDashboard.putNumber("Speed Error Rad Per Sec", Math.abs(inputs.velocityRadPerSec - inputs.velocitySetpointRadPerSec));
        double endTime = Logger.getInstance().getRealTimestamp();
        Logger.getInstance().recordOutput("LauncherCodeSec", endTime - startTime);
    }

    public void spinFromDistance(double distance) {
        spinLauncher(interpolationTable.get(distance));
    }
}

