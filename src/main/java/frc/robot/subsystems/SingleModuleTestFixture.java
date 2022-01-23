// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.TestFixtureConstants;
import lib.Loggable;
import badlog.lib.BadLog;


public class SingleModuleTestFixture extends SubsystemBase implements Loggable {
  /** Creates a new SingleModuleTestFixture. */

  private final WPI_TalonFX driveMotor = new WPI_TalonFX(TestFixtureConstants.kDriveMotorPort);
  private final WPI_TalonFX turningMotor = new WPI_TalonFX(TestFixtureConstants.kTurningMotorPort);
  private final CANCoder encoder = new CANCoder(TestFixtureConstants.kCANCoderID);
  private final PIDController m_turningPIDController = 
      new PIDController(
          ModuleConstants.kPModuleTurningController,
          0,
          0
          // new TrapezoidProfile.Constraints(
          //     ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond,
          //     ModuleConstants.kMaxModuleAngularAccelerationRadiansPerSecondSquared)
          );

  private Rotation2d swerveAngle;
  private final PIDController velocityPIDController = new PIDController(0.1, 0, 0);

  public SingleModuleTestFixture() {
    driveMotor.setNeutralMode(NeutralMode.Brake);
    turningMotor.setNeutralMode(NeutralMode.Brake);
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    swerveAngle = new Rotation2d(0,1);
    SmartDashboard.putNumber("P Gain Input", 0);
    SmartDashboard.putNumber("I Gain Input", 0);
    SmartDashboard.putNumber("D Gain Input", 0);
    SmartDashboard.putNumber("Angle Setpoint Degrees", 0);


  }

  public void setState(double driveSpeed, double turnSpeed) {
    driveMotor.set(driveSpeed/2);
    System.out.println("Setting Drive Motor To "+driveSpeed);
    // turningMotor.set(turnSpeed/2);
    // System.out.println("Setting Turning Motor To "+driveSpeed);
  }

  public void setAngle(Rotation2d desiredAngle) {
    swerveAngle = desiredAngle;
  }


  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(encoder.getAbsolutePosition());
  }

  public Rotation2d getError(Rotation2d actual, Rotation2d desired) {
    return desired.minus(actual);
  }

  public void syncPIDGains() {
    m_turningPIDController.setP(SmartDashboard.getNumber("P Gain Input", 0));
    m_turningPIDController.setI(SmartDashboard.getNumber("I Gain Input", 0));
    m_turningPIDController.setD(SmartDashboard.getNumber("D Gain Input", 0));
    swerveAngle = new Rotation2d(SmartDashboard.getNumber("Angle Setpoint Degrees", 0));
  }

  @Override
  public void periodic() {
    System.out.println(this.getAngle());
    syncPIDGains();
    SmartDashboard.putNumber("SwerveAngle", this.getAngle().getRadians());
    SmartDashboard.putNumber("PIDSetpoint", m_turningPIDController.getSetpoint());
    SmartDashboard.putNumber("PIDInput", m_turningPIDController.getPositionError());
    SmartDashboard.putNumber("P Gain", m_turningPIDController.getP());
    SmartDashboard.putNumber("D Gain", m_turningPIDController.getD());
    SmartDashboard.putNumber("I Gain", m_turningPIDController.getI());
    final var turnOutput =
        m_turningPIDController.calculate(0, getError(getAngle(),swerveAngle).getRadians());
    turningMotor.set(turnOutput);
    SmartDashboard.putNumber("Turn Output", turnOutput);
    SmartDashboard.putNumber("Computed Error", getError(getAngle(),swerveAngle).getRadians());
    SmartDashboard.putNumber("Steering PID Position Error", m_turningPIDController.getPositionError());
    
    // final double velocity = velocityPIDController.calculate((driveMotor.getSelectedSensorVelocity()/204.8), TestFixtureConstants.kTestVelocity);
    // driveMotor.set(velocity);
    // This method will be called once per scheduler run
  }

  public void logInit() {
    BadLog.createValue("P Gain", ""+m_turningPIDController.getP());
    BadLog.createValue("I Gain", ""+m_turningPIDController.getI());
    BadLog.createValue("D Gain", ""+m_turningPIDController.getD());

    BadLog.createTopic("Steering Rotation Angle Degrees", "degrees", () -> getAngle().getDegrees());
    BadLog.createTopic("Steering Rotation Angle Radians", "rad", () -> getAngle().getRadians());

    BadLog.createTopic("Steering PID Position Error", "rad", () -> m_turningPIDController.getPositionError(), "join:Swerve Steering PID Control");
    BadLog.createTopic("Steering PID Setpoint", "rad", () -> m_turningPIDController.getSetpoint(), "join:Swerve Steering PID Control");
    BadLog.createTopic("Steering Motor Output", "PercentOutput", () -> turningMotor.get(), "join:Swerve Steering PID Control");

  }
}