// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Limelight;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    //NetworkTableEntry ty = table.getEntry("ty");

    double x = tx.getDouble(0.0);
    //double y = ty.getDouble(0.0);
    
    SmartDashboard.putNumber("LimelightX", x);
    //SmartDashboard.putNumber("LimelightY", y);
    }

  @Override
  public void periodic() {
    double startTime = Logger.getInstance().getRealTimestamp();
    horizontalAngle = tx.getDouble(horizontalAngle);
    if (Math.abs(ty.getDouble(verticalAngle))>0) {
      verticalAngle = ty.getDouble(verticalAngle);
      double endTime = Logger.getInstance().getRealTimestamp();
      Logger.getInstance().recordOutput("LimelighteCodeSec", endTime-startTime);
    }
    
    SmartDashboard.putNumber("LimelightX", horizontalAngle);
    SmartDashboard.putNumber("LimelightY", verticalAngle);
    System.out.println("Running Limelight Periodic");
    SmartDashboard.putNumber("Distanceawayfromtarget", (Constants.LauncherConstants.heightOfHighHubReflectors/(Math.tan(Units.degreesToRadians(getVerticalAngle()+30)))));
    System.out.println("Distance away from target: "+(2.64/(Math.tan(Units.degreesToRadians(getVerticalAngle()+30)))));
    // This method will be called once per scheduler run
  }
}
