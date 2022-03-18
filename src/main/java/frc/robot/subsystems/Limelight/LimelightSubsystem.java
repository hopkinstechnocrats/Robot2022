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
  NetworkTableEntry tv;
  NetworkTableEntry ty;
  NetworkTableEntry tx;
  double horizontalAngle;
  double verticalAngle;
  double P = 20;
  double I, D = 0;
  double period = 0.1;
  PIDController aiming = new PIDController(P, I, D, period);

  public LimelightSubsystem() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    tv = table.getEntry("tv");
    horizontalAngle = tx.getDouble(0.0);
    verticalAngle = ty.getDouble(0.0);
    SmartDashboard.putNumber("LimelightX", horizontalAngle);
    SmartDashboard.putNumber("LimelightY", verticalAngle);
  }

  @Override
  public void periodic() { 
    // This method will be called once per scheduler run
  }

  public boolean isTargetVisible(){
    if (tv.getDouble(0) == 1){
      return true;
    }
    else {
      return false;
    }
    
  }

  public double getRotationSpeed(){
    System.out.println(tx.getDouble(0));
    return aiming.calculate(tx.getDouble(0), 0);
    
  }

  public double getVerticalAngle() {
    return verticalAngle;
  }
}
