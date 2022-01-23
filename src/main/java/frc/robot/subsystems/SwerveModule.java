// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;
import lib.Loggable;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import badlog.lib.BadLog;

public class SwerveModule {
  WPI_TalonFX driveMotor;
  WPI_TalonFX turningMotor;
  CANCoder encoder; 
  String corners;


  private final PIDController m_drivePIDController =
      new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

  private final PIDController m_turningPIDController = 
      new PIDController(
        ModuleConstants.kPModuleTurningController,
        0,
        0
        );
        

  private Rotation2d swerveAngle;
  private double offset;
  private boolean inverted;

  public SwerveModule(int DriveMotorReport, 
  int TurningMotorReport, int DriveEncoderReport,
   int TurningEncoderReport, boolean DriveEncoderReversed, 
    boolean TurningEncoderReversed, String corners, double turningEncoderOffset){

    this.corners = corners;
    this.offset = turningEncoderOffset;
    encoder = new CANCoder(TurningEncoderReport);

    driveMotor = new WPI_TalonFX(DriveMotorReport);
    turningMotor = new WPI_TalonFX(TurningMotorReport);
    final CANCoder encoder = new CANCoder(TurningEncoderReport);
    driveMotor.setNeutralMode(NeutralMode.Brake);
    turningMotor.setNeutralMode(NeutralMode.Brake);
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    swerveAngle = new Rotation2d(0,1);
    SmartDashboard.putNumber(corners + "P Gain Input", 0);
    SmartDashboard.putNumber(corners + "I Gain Input", 0);
    SmartDashboard.putNumber(corners + "D Gain Input", 0);
    SmartDashboard.putNumber(corners + "Angle Setpoint Degrees", 0);
    
  }

  public void periodic() {
    SmartDashboard.putNumber(corners + "SwerveAngle", this.getAngle().getRadians());
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(encoder.getAbsolutePosition()).minus(new Rotation2d(offset));
  }

  public Rotation2d getError(Rotation2d actual, Rotation2d desired) {
    return desired.minus(actual);
  }

  public void setInverted(boolean inverted) {
    this.inverted = inverted;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState((driveMotor.getSelectedSensorVelocity()/ModuleConstants.kEncoderTicksPerRevolution*10)*Constants.DriveConstants.kWheelHeight*Math.PI, getAngle());
  }
  //

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = desiredState;
    // SwerveModuleState.optimize(desiredState, getAngle());

    // Calculate the drive output from the drive PID controller.
    final double driveOutput = inverted ?
        m_drivePIDController.calculate(getState().speedMetersPerSecond, desiredState.speedMetersPerSecond)
        : -1*m_drivePIDController.calculate(getState().speedMetersPerSecond, desiredState.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final Double turnOutput =
        m_turningPIDController.calculate(getAngle().getRadians(), state.angle.getRadians());

    // Calculate the turning motor output from the turning PID controller.
    driveMotor.setVoltage(driveOutput);
    turningMotor.setVoltage(turnOutput);

    SmartDashboard.putNumber(corners + "Drive Output", driveOutput);
    SmartDashboard.putNumber(corners + "Measuerd Output", getState().speedMetersPerSecond);
    SmartDashboard.putNumber(corners + "Turn Output", turnOutput);
    SmartDashboard.putNumber(corners + "Desired State m/s", desiredState.speedMetersPerSecond);
    SmartDashboard.putNumber(corners + "SwerveAngle", this.getAngle().getRadians());
    SmartDashboard.putNumber(corners + "PIDSetpoint", m_turningPIDController.getSetpoint());
    SmartDashboard.putNumber(corners + "PIDInput", m_turningPIDController.getPositionError());
    SmartDashboard.putNumber(corners + "P Gain", m_turningPIDController.getP());
    SmartDashboard.putNumber(corners + "D Gain", m_turningPIDController.getD());
    SmartDashboard.putNumber(corners + "I Gain", m_turningPIDController.getI());
    SmartDashboard.putNumber(corners + "Drive P Gain", m_drivePIDController.getP());
    m_drivePIDController.setP(SmartDashboard.getNumber("PID P Gain Input", 0));
    SmartDashboard.putNumber(corners + "Computed Error", getError(getAngle(),swerveAngle).getRadians());
    SmartDashboard.putNumber(corners + "Steering PID Position Error", m_turningPIDController.getPositionError());
  }

  /** Zeros all the SwerveModule encoders. */
  public void resetEncoders() {

  }

  public void logInit() {
    BadLog.createValue(corners + "P Gain", ""+m_turningPIDController.getP());
    BadLog.createValue(corners + "I Gain", ""+m_turningPIDController.getI());
    BadLog.createValue(corners + "D Gain", ""+m_turningPIDController.getD());

    BadLog.createTopic(corners + "Steering Rotation Angle Degrees", "degrees", () -> getAngle().getDegrees());
    BadLog.createTopic(corners + "Steering Rotation Angle Radians", "rad", () -> getAngle().getRadians());

    BadLog.createTopic(corners + "Steering PID Position Error", "rad", () -> m_turningPIDController.getPositionError(), "join:Swerve Steering PID Control");
    BadLog.createTopic(corners + "Steering PID Setpoint", "rad", () -> m_turningPIDController.getSetpoint(), "join:Swerve Steering PID Control");
    BadLog.createTopic(corners + "Steering Motor Output", "PercentOutput", () -> turningMotor.get(), "join:Swerve Steering PID Control");

  }
}
