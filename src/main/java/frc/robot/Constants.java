// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

/** Add your docs here. */
public final class Constants {
  public static final RobotType robotType = RobotType.TinmanV1;
  public static final double autoAlignDistanceThreshold = Meters.of(.1).in(Meters);
  public static final double targetDistanceToHub = Meters.of(2).in(Meters);
  public static String middleLimeLight = "limelight-front";
  public static String drivebaseCanbusName = "Drivebase";
  public static final CANBus superstructureCanbus = CANBus.roboRIO();

  public static class Collector {
    public static final int kMotorId1 = 14;
    public static final int kMotorId2 = 15;
    public static final int kMotorId3 = 16;
    public static final int kMotorId4 = 17;
    public static final double kGearRatio = 2;

    public static Slot0Configs slot0() {
      Slot0Configs slot0Configs = new Slot0Configs();
      slot0Configs.kP = .06;
      slot0Configs.kS = .1;
      return slot0Configs;
    }
  };

  public static class Feeder {
    public static final int kMotorId1 = 18;
    public static final double kGearRatio = 1;

    public static Slot0Configs slot0() {
      Slot0Configs slot0Configs = new Slot0Configs();
      slot0Configs.kP = 0.11;
      slot0Configs.kS = 0.60;
      return slot0Configs;
    }
  };

  public static class Shooter {
    public static final int kLeaderMotorId = 19;
    public static final int kFollowerMotorId = 20;
    public static final double kGearRatio = 1;

    public static TalonFXConfiguration configs() {
      var config = new TalonFXConfiguration();
      config.Slot0.kP = 999999.0;
      config.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
      config.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;
      config.MotorOutput.PeakForwardDutyCycle = 1.0;
      config.MotorOutput.PeakReverseDutyCycle = 0.0;
      return config;
    }
  };

  public static class Spindexer {
    public static final int kMotorId1 = 21;
    public static final double kGearRatio = 6;
    public static final double spindexerIdleSpeed = 2;

    public static Slot0Configs slot0() {
      Slot0Configs slot0Configs = new Slot0Configs();
      slot0Configs.kP = .06;
      slot0Configs.kS = .1;
      return slot0Configs;
    }
  };

  public enum RobotType {
    TinmanV0,
    TinmanV1
  }
}
