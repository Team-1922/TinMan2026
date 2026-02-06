// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

/** Add your docs here. */
public final class Constants {

    public static double offsetInMeters;
    public static double targetDistanceToTag;
    public static final RobotType robotType = RobotType.Tinman1;
    public static String middleLimeLight = "limelight-front";
    public static String canbus = "Drivebase";

    public enum RobotType{
        Tinman1,
        Tinman2
    }

}
