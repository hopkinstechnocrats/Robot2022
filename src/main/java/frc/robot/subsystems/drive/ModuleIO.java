package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class ModuleIO {
    WPI_TalonFX driveMotor;

    CANCoder encoder;




    private final PIDController m_drivePIDController =
            new PIDController(Constants.ModuleConstants.kPModuleDriveController, 0, 0);

    public ModuleIO(int DriveMotorReport,
                    int TurningMotorReport, int DriveEncoderReport,
                    int TurningEncoderReport, boolean DriveEncoderReversed,
                    boolean TurningEncoderReversed, double turningEncoderOffset) {

        driveMotor = new WPI_TalonFX(DriveMotorReport);
        turningMotor = new WPI_TalonFX(TurningMotorReport);
        final CANCoder encoder = new CANCoder(TurningEncoderReport);
        driveMotor.setNeutralMode(NeutralMode.Brake);
        turningMotor.setNeutralMode(NeutralMode.Brake);
        m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void updateInputs(ModuleIOInputs inputs) {
        inputs.steerInputs.
    }
}
