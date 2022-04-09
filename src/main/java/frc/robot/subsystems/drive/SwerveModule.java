// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
    driveIO = new ModuleDriveIO(driveMotorPort, true, corners);
    steerInputs = new ClosedLoopIO.ClosedLoopIOInputs(1);
    driveInputs = new ClosedLoopIO.ClosedLoopIOInputs(1);
    desiredState = new SwerveModuleState(0, new Rotation2d(0));
    this.corners = corners;
    this.offset = turningEncoderOffset;
  }

  public void periodic() {
    // double startTime = Logger.getInstance().getRealTimestamp();
    steerIO.updateInputs(steerInputs);
    driveIO.updateInputs(driveInputs);
    // double elapsed = Logger.getInstance().getRealTimestamp() - startTime;
    // Logger.getInstance().recordOutput("DriveIOInputUpdateSec", elapsed);

    // Logger.getInstance().processInputs("DriveSubsystem/" + corners + " Steer", steerInputs);
    // Logger.getInstance().processInputs("DriveSubsystem/" + corners + " Drive", driveInputs);

    // startTime = Logger.getInstance().getRealTimestamp();
    desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
    steerIO.setPosition(desiredState.angle);
    driveIO.setVelocityRadPerSec(desiredState.speedMetersPerSecond / (kWheelHeight / 2));
    // elapsed = Logger.getInstance().getRealTimestamp();
    // Logger.getInstance().recordOutput("DriveIOSetpointUpdateSec", elapsed);

    // Logger.getInstance().recordOutput("DriveSubsystem/" + corners + " ModuleAngleRad", getState().angle.getRadians());
    // Logger.getInstance().recordOutput("DriveSubsystem/" + corners + " ModuleSpeedMetersPerSecond", getState().speedMetersPerSecond);
    // Logger.getInstance().recordOutput("DriveSubsystem/" + corners + " DesiredModuleAngleRad", desiredState.angle.getRadians());
    // Logger.getInstance().recordOutput("DriveSubsystem/" + corners + " DesiredModuleSpeedMetersPerSecond", desiredState.speedMetersPerSecond);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
            driveInputs.velocityRadPerSec * (kWheelHeight / 2),
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
