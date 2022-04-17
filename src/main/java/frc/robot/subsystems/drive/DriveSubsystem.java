// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SerialPort;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.MathUtil;

public class DriveSubsystem extends SubsystemBase {
  boolean fieldOriented = false;
  private int negate = 1;
  // Robot swerve modules
  private final SwerveModule m_frontLeft =
      new SwerveModule(
          DriveConstants.kFrontLeftDriveMotorPort,
          DriveConstants.kFrontLeftTurningMotorPort,
              DriveConstants.kFrontLeftTurningEncoderPort,
              "FrontLeft", DriveConstants.kFrontLeftOffset);


  private final SwerveModule m_rearLeft =
      new SwerveModule(
          DriveConstants.kRearLeftDriveMotorPort,
          DriveConstants.kRearLeftTurningMotorPort,
              DriveConstants.kRearLeftTurningEncoderPort,
              "RearLeft", DriveConstants.kRearLeftOffset);

  private final SwerveModule m_frontRight =
      new SwerveModule(
          DriveConstants.kFrontRightDriveMotorPort,
          DriveConstants.kFrontRightTurningMotorPort,
              DriveConstants.kFrontRightTurningEncoderPort,
              "FrontRight", DriveConstants.kFrontRightOffset);

  private final SwerveModule m_rearRight =
      new SwerveModule(
          DriveConstants.kRearRightDriveMotorPort,
          DriveConstants.kRearRightTurningMotorPort,
              DriveConstants.kRearRightTurningEncoderPort,
              "RearRight", DriveConstants.kRearRightOffset);
  
              Rotation2d targetAngle;

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SerialPort.Port.kMXP);

  //field
  private final Field2d m_field = new Field2d();

  private final SlewRateLimiter xSpeedFilter = new SlewRateLimiter(5);
  private final SlewRateLimiter ySpeedFilter = new SlewRateLimiter(5);
  private final SlewRateLimiter rotFilter = new SlewRateLimiter(25);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
     new SwerveDriveOdometry(DriveConstants.kDriveKinematics, m_gyro.getRotation2d());

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    SmartDashboard.putNumber("PID P Gain Input", 0);
    SmartDashboard.putData("field", m_field);
  }

  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
      getHeading(),
      m_frontLeft.getState(),
      m_rearLeft.getState(),
      m_frontRight.getState(),
      m_rearRight.getState());
    double startTime = Logger.getInstance().getRealTimestamp();
    m_frontLeft.periodic();
    m_frontRight.periodic();
    m_rearLeft.periodic();
    m_rearRight.periodic();
    double endTime = Logger.getInstance().getRealTimestamp();
    // Logger.getInstance().recordOutput("Roll", (double) getRoll());
    // SmartDashboard.putNumber("Heading", getHeading().getDegrees());
    // SmartDashboard.putNumber("KPTurningController", 4);
    // SmartDashboard.putNumber("KITurningController", 64);
    // SmartDashboard.putNumber("KDTurningController", 0.01);
    m_field.setRobotPose(getPose());
    Pose2d transformedPose = getPose().transformBy(new Transform2d(new Translation2d(Units.feetToMeters(27),0), new Rotation2d(0))); 
    Logger.getInstance().recordOutput("Odometry/RobotPose",
            new double[] {getPose().getX(), getPose().getY(), getPose().getRotation().getRadians()});
    Logger.getInstance().recordOutput("PeriodicCodeSec", endTime-startTime);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
   public Pose2d getPose() {
     return m_odometry.getPoseMeters();
   }
   public void setTargetAngle(Rotation2d targetAngle){
    this.targetAngle = targetAngle;
   }

   public Rotation2d getTargetAngle(){
    return targetAngle;
}

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
   public void resetOdometry(Pose2d pose) {
     m_odometry.resetPosition(pose, getHeading());
   }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot) {
    Logger.getInstance().recordOutput("DriveSubsystem/Raw Rotation Command", rot);
    
    rot =  -1*MathUtil.applyDeadband(rot, 0.4);
    ySpeed = negate*MathUtil.applyDeadband(ySpeed, 0.2);
    xSpeed =  negate*MathUtil.applyDeadband(xSpeed, 0.2);
    rot = rotFilter.calculate(rot);
    ySpeed = ySpeedFilter.calculate(ySpeed);
    xSpeed = xSpeedFilter.calculate(xSpeed);
    
    driveNoDeadband(xSpeed, ySpeed, rot);
  }

  public void driveNoDeadband(double xSpeed, double ySpeed, double rot) {
  
    
    Logger.getInstance().recordOutput("DriveSubsystem/Rotation Command", rot);
    Logger.getInstance().recordOutput("DriveSubsystem/xSpeed Command", xSpeed);
    Logger.getInstance().recordOutput("DriveSubsystem/ySpeed Command", ySpeed);

    SwerveModuleState[] swerveModuleStates;
    if (fieldOriented) {
      swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
          // fieldRelative
          ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d()));
    } else{
      swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
          new ChassisSpeeds(xSpeed, ySpeed, rot));
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_rearLeft.setDesiredState(swerveModuleStates[1]);
    m_frontRight.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_rearLeft.setDesiredState(desiredStates[1]);
    m_frontRight.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.zeroYaw();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-1*m_gyro.getFusedHeading());
  }

  public Rotation2d setRotation() {
    return new Rotation2d(Math.PI/2);
  }

  public void fieldON() {
    fieldOriented = true;
    driveNoDeadband(0, 0, 0);
  }

  public void fieldOFF() {
    fieldOriented = false;
    driveNoDeadband(0, 0, 0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate();
  }

public float getRoll() {
    return -1* m_gyro.getRoll();
}
public void makeBackwards(boolean GOGOGOGOGOGOGO) {
  if (GOGOGOGOGOGOGO){
    negate = -1;
  } else{
    negate = 1;
  }
}
}
