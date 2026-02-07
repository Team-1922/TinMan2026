// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

/** Add your docs here. */
public final class Constants {

    public static double offsetInMeters = Meters.of(.5).in(Meters);
    public static double targetDistanceToTag = Meters.of(2.7).in(Meters);
    public static final RobotType robotType = RobotType.TinmanV0;
    public static String middleLimeLight = "limelight-front";
    public static String drivebaseCanbusName = "Drivebase";

    public static int collectorID1 = 14;
    public static int collectorID2 = 15;
    public static int collectorID3 = 16;
    public static int collectorID4 = 17;

    public static int feederID = 18;

    public static int shooterMotorID1 = 19;
    public static int shooterMotorID2 = 20;

    public static int spindexerID = 21;

    public enum RobotType{
        TinmanV0,
        TinmanV1
    }
}
