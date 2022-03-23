// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class FieldPositions {
    //origin is the center of the hub. The positive x value was towards the blue side and positive y values was towards the scoring tables.
    public static final Translation2d B1 = new Translation2d(0.658, 3.830);
    public static final Translation2d B2 = new Translation2d(3.174, 2.243);
    public static final Translation2d B3 = new Translation2d(3.287, -2.074);
    public static final Translation2d B4 = new Translation2d(0.858, -3.790);
    public static final Translation2d B5 = new Translation2d(-3.790, -0.858);
    public static final Translation2d B6 = new Translation2d(-2.243, 3.174);
    public static final Translation2d B7 = new Translation2d(7.165, 2.990);
    public static final Translation2d B8 = new Translation2d(7.165, 2.990); //In human player's hand. No definite coordinates.
    public static final Translation2d R1 = new Translation2d(-0.658, -3.830);
    public static final Translation2d R2 = new Translation2d(-3.174, -2.243);
    public static final Translation2d R3 = new Translation2d(-3.287, 2.074);
    public static final Translation2d R4 = new Translation2d(-0.858, 3.790);
    public static final Translation2d R5 = new Translation2d(3.790, 0.858);
    public static final Translation2d R6 = new Translation2d(2.243, -3.174);
    public static final Translation2d R7 = new Translation2d(-7.165, -2.990);
    public static final Pose2d R3startingPosition = new Pose2d(- 1.0, 1.0, new Rotation2d()); //measure for real dimensions before testing
    public static final Translation2d R8 = new Translation2d(-7.165, -2.990);  //In human player's hand. No definite coordinates.
    public static final Translation2d redFarHangarCorner = new Translation2d(-8.220, 4.121); 
    public static final Translation2d redDisposalLocation = new Translation2d(-8.220 + 0.5, 4.121 - 0.5); //variable location for disposal of opponent's balls
    public static final Translation2d blueFarHangarCorner = new Translation2d(8.220, -4.121); 
    public static final Translation2d blueDisposalLocation = new Translation2d(8.220 - 0.5, -4.121 + 0.5); //variable location for disposal of opponent's balls

}
