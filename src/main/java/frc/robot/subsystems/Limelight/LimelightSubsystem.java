// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Limelight;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem extends SubsystemBase {
  /** Creates a new LimelightSubsystem. */
  NetworkTableEntry tv;
  NetworkTableEntry ty;
  NetworkTableEntry tx;
  double horizontalAngle = 0;
  double verticalAngle = 0;
  double isTargetVisible = 0;
  double P = 0.15;
  double I = 0;
  double D = 0;
  double period = 0.1;
  PIDController aiming = new PIDController(P, I, D, period);

  public LimelightSubsystem() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    tv = table.getEntry("tv");
    
  }

  @Override
  public void periodic() {
    
    
    
    // SmartDashboard.putNumber("LimelightX", horizontalAngle);
    // SmartDashboard.putNumber("LimelightY", verticalAngle);
    // System.out.println("Running Limelight Periodic");
    // SmartDashboard.putNumber("Distanceawayfromtarget", (Constants.LauncherConstants.heightOfHighHubReflectors/(Math.tan(Units.degreesToRadians(getVerticalAngle()+30)))));
    // System.out.println("Distance away from target: "+(2.64/(Math.tan(Units.degreesToRadians(getVerticalAngle()+30)))));
    // // This method will be called once per scheduler run
  }

  public boolean isTargetVisible(){
    isTargetVisible = tv.getDouble(isTargetVisible);
    if (isTargetVisible == 1){
      return true;
    }
    else {
      return false;
    }
    
  }

  public double getRotationSpeed(){
    if (Math.abs(tx.getDouble(horizontalAngle))>0.5) {
      horizontalAngle = tx.getDouble(horizontalAngle);
    }
    Logger.getInstance().recordOutput("Limelight/horizontalAngle", horizontalAngle);
    Logger.getInstance().recordOutput("Limelight/RotationSpeed",aiming.calculate(horizontalAngle, 0));
    return aiming.calculate(horizontalAngle, 0);
  }

  public double getVerticalAngle() {
    if (Math.abs(ty.getDouble(verticalAngle))>0) {
      verticalAngle = ty.getDouble(verticalAngle);
    }
    return verticalAngle;
  }
}
