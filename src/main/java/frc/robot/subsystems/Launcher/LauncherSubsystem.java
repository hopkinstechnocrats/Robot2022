package frc.robot.subsystems.Launcher;


import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;
import lib.LinearInterpolator;
import lib.iotemplates.ClosedLoopIO;
import lib.iotemplates.OpenLoopIO;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import lib.iotemplates.VelocityClosedLoopIOTalon;
import lib.util.TunableNumber;

import org.littletonrobotics.junction.Logger;

public class LauncherSubsystem extends SubsystemBase {

    private final ClosedLoopIO io;
    private final ClosedLoopIO.ClosedLoopIOInputs inputs = new ClosedLoopIO.ClosedLoopIOInputs(2);
    private final LinearInterpolator interpolationTable;
    XboxController m_driverController;
    XboxController m_operator;
    private final LEDSubsystem m_led;
    TunableNumber spoid = new TunableNumber("Launcher/speed", 4000);
    TunableNumber scalingFactor = new TunableNumber("Launcher/ScalingFactor", 1);

    public LauncherSubsystem(LEDSubsystem led, XboxController m_driverController, XboxController m_operator) {
        io = new VelocityClosedLoopIOTalon(
                "launcher",
                new int[] {Constants.LauncherConstants.kCANPort1, Constants.LauncherConstants.kCANPort2},
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
        this.m_driverController = m_driverController;
        this.m_operator = m_operator;
        //interpolationTable.put()
        
    }

    public LauncherSubsystem(LEDSubsystem led, XboxController m_driverController, XboxController m_operator, ClosedLoopIO io) {
        this.io = io;
        interpolationTable = new LinearInterpolator();

        // interpolationTable.put(0, 0);
        interpolationTable.put(3.08, 6000);
        interpolationTable.put(5.69, 6904);
        interpolationTable.put(7.80187249, 7565);
        m_led = led;
        this.m_driverController = m_driverController;
        this.m_operator = m_operator;
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

        if (Math.abs(inputs.velocityRadPerSec - inputs.velocitySetpointRadPerSec)<20) {
            m_operator.setRumble(RumbleType.kLeftRumble, 1);
        } else {
            m_operator.setRumble(RumbleType.kLeftRumble, 0);
        }
    }

    public Boolean deadzoneIn() {
        if(Math.abs(inputs.velocityRadPerSec - inputs.velocitySetpointRadPerSec)<50) {
            return true;
        } else {
            return false;
        }
    }

    public Boolean deadzoneOut() {
        if(Math.abs(inputs.velocityRadPerSec - inputs.velocitySetpointRadPerSec)>50) {
            return true;
        } else {
            return false;
        }
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
        SmartDashboard.putBoolean("At Speed", Math.abs(inputs.velocityRadPerSec - inputs.velocitySetpointRadPerSec)<20);
        SmartDashboard.putNumber("Speed Error Rad Per Sec", Math.abs(inputs.velocityRadPerSec - inputs.velocitySetpointRadPerSec));
       double endTime = Logger.getInstance().getRealTimestamp();
       Logger.getInstance().recordOutput("LauncherCodeSec", endTime-startTime);
    }

    public void spinFromDistance(double distance) {
        spinLauncher(interpolationTable.get(distance));
    }
}

