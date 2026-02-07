// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import frc.robot.subsystems.Spindexer;

/** Add your docs here. */
public final class Constants {

    public static final double offsetInMeters = Meters.of(.5).in(Meters);
    public static final double targetDistanceToTag = Meters.of(2.7).in(Meters);
    public static final RobotType robotType = RobotType.TinmanV0;
    public static String middleLimeLight = "limelight-front";
    public static String drivebaseCanbusName = "Drivebase";

    public static class Collector {
        public static final int kMotorId1 = 14;
        public static final int kMotorId2 = 15;
        public static final int kMotorId3 = 16;
        public static final int kMotorId4 = 17;
    };

    public static class Feeder {
        public static final int kMotorId1 = 18;
    };

    public static class Shooter{
        public static final int kMotorId1 = 19;
        public static final int kMotorId2 = 20;
    };

    public static class Spindexer {
        public static final int kMotorId1 = 21;
    };

    public enum RobotType{
        TinmanV0,
        TinmanV1
    }
}
