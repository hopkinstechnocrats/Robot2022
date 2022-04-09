// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  DigitalOutput output;
  int intaking;
  int climbing;
  int warning;
  int launching;
  int onTarget;
  int allianceColor;
  double proportionOn; // same as duty cycle
  
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    output = new DigitalOutput(0);
    proportionOn = 0;
    
    // Initial duty cycle is 0 so it's the same as when it's off
    output.enablePWM(0);
  }

  @Override
  public void periodic() {
    proportionOn = convertToDutyCycle(determineOutput());
    output.updateDutyCycle(proportionOn);
        // This method will be called once per scheduler run

    if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
      teamColorBlue();
    } else if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
      teamColorRed();
    }
  }

  public int determineOutput() {
    return onTarget + 2 * warning + 2^2 * climbing + 2^3 * intaking + 2^4 * launching + 2^5 * allianceColor; // if we want the plus one we can keep it+ 1 // the plus one is to make it so the cases range from 1-64 instead of 0-63
  }

  public double convertToDutyCycle(int val) {
    return (val/64);
  }

  public void intakeOn() {
    intaking = 1;
  }

  public void intakeOff() {
    intaking = 0;
  }

  public void launchingOn() {
    launching = 1;
  }

  public void launchingOff() {
    launching = 0;
  }

  public void climbingOn() {
    climbing = 1;
  }

  public void climbingOff() {
    climbing = 0;
  }

  public void warningOn() {
    warning = 1;
  }

  public void warningOff() {
    warning = 0;
  }

  public void onTargetOn() {
    onTarget = 1;
  }

  public void onTargetOff() {
    onTarget = 0;
  }

  public void teamColorBlue() {
    allianceColor = 0;
  }

  public void teamColorRed() {
    allianceColor = 1;
  }
}
