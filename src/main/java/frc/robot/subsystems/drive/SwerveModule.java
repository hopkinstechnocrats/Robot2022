// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import badlog.lib.BadLog;
import lib.iotemplates.ClosedLoopIO;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.DriveConstants.kWheelHeight;

public class SwerveModule {

  private String corners;

  private Rotation2d swerveAngle;
  private double offset;
  private boolean inverted;

  private final ModuleSteerIO steerIO;
  private final ModuleDriveIO driveIO;
  private SwerveModuleState desiredState;
  private final ClosedLoopIO.ClosedLoopIOInputs steerInputs;
  private final ClosedLoopIO.ClosedLoopIOInputs driveInputs;

  public SwerveModule(int driveMotorPort,
                      int turningMotorPort,
                      int turningEncoderPort,
                      String corners, double turningEncoderOffset){

    steerIO = new ModuleSteerIO(turningMotorPort, turningEncoderPort, turningEncoderOffset);
    driveIO = new ModuleDriveIO(driveMotorPort, false);
    steerInputs = new ClosedLoopIO.ClosedLoopIOInputs();
    driveInputs = new ClosedLoopIO.ClosedLoopIOInputs();
    this.corners = corners;
    this.offset = turningEncoderOffset;
  }

  public void periodic() {
    steerIO.updateInputs(steerInputs);
    driveIO.updateInputs(driveInputs);

    Logger.getInstance().processInputs("DriveSubsystem/" + corners + " Steer", steerInputs);
    Logger.getInstance().processInputs("DriveSubsystem/" + corners + " Drive", driveInputs);

    steerIO.setPosition(desiredState.angle);
    driveIO.setVelocity(desiredState.speedMetersPerSecond);

    Logger.getInstance().recordOutput("DriveSubsystem/" + corners + " ModuleAngleRad", getState().angle.getRadians());
    Logger.getInstance().recordOutput("DriveSubsystem/" + corners + " ModuleSpeedMetersPerSecond", getState().speedMetersPerSecond);
    Logger.getInstance().recordOutput("DriveSubsystem/" + corners + " DesiredModuleAngleRad", desiredState.angle.getRadians());
    Logger.getInstance().recordOutput("DriveSubsystem/" + corners + " DesiredModuleSpeedMetersPerSecond", desiredState.angle.getRadians());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
            driveInputs.velocityRadPerSec * kWheelHeight / 2,
            new Rotation2d(steerInputs.positionRad)
    );
  }
  //

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    this.desiredState = desiredState;
  }
}
