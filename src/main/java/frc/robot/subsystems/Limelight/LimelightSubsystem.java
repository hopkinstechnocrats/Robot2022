// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Limelight;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */
  double P = 1;
  double I, D = 0;
  double period = 0.1;
  PIDController aiming = new PIDController(P, I, D, period);

  public LimelightSubsystem() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    //NetworkTableEntry ty = table.getEntry("ty");
    

    double horizontalAngle = tx.getDouble(0.0);
    //double y = ty.getDouble(0.0);
    
    SmartDashboard.putNumber("LimelightX", horizontalAngle);
    //SmartDashboard.putNumber("LimelightY", y);
    }

  @Override
  public void periodic() { 
    // This method will be called once per scheduler run
  }
}
