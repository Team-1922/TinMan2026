// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.generated.TunerConstants;


/** Add your docs here. */
public final class Constants {

    public static final RobotType robotType = RobotType.TinmanV2;

    public static final double autoAlignDistanceThreshold = 
        Meters.of(.1).in(Meters);
    public static final double maxTargetDistanceToHub = Meters.of(4.3).in(Meters);
    public static String middleLimeLight = "limelight-front";
    public static String drivebaseCanbusName = "Drivebase";
    public static final CANBus superstructureCanbus =  switch(Constants.robotType) {
            case TinmanV2 -> TunerConstants.kCANBus;
            case TinmanV1 ->  CANBus.roboRIO();
        };

    public static class Collector {
        public static final int kRollerMotorId = 14;
        public static final int kPivotMotorId = 16;
        public static final int kPivotCanCoderId = 22;
        public static final double kRollerGearRatio = 2;
        public static final double krps = 100;

        public static Slot0Configs slot0() {
            Slot0Configs slot0Configs = new Slot0Configs();
            slot0Configs.kP = .06;
            slot0Configs.kS = .1;
            return slot0Configs;
        }

        public static Slot0Configs pivotSlot0() {
            Slot0Configs slot0Configs = new Slot0Configs();
            slot0Configs.kP = 2;
            slot0Configs.kS = 0;
            return slot0Configs;
        }


        public static final CurrentLimitsConfigs kRollerCurrentConfigs = 
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(20)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(30)
                .withSupplyCurrentLowerLimit(20)
                .withSupplyCurrentLowerTime(.75);

        public static final CurrentLimitsConfigs kPivotCurrentConfigs = 
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(20)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(20)
                .withSupplyCurrentLowerLimit(20)
                .withSupplyCurrentLowerTime(.75);

        public static final FeedbackConfigs kPivotFeedbackConfig = new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
            .withFeedbackRemoteSensorID(kPivotCanCoderId)
            .withRotorToSensorRatio(48)
            .withSensorToMechanismRatio(1);

        public static final CANcoderConfiguration kPivotCanCoderConfig =
            new CANcoderConfiguration().withMagnetSensor(
                    new MagnetSensorConfigs()
                        .withMagnetOffset(0.166748)
                        .withAbsoluteSensorDiscontinuityPoint(0.5)
                        .withSensorDirection(
                                SensorDirectionValue.CounterClockwise_Positive
                        )
            );

        public static final double kRetractedPosition = 0.375;

        public static final double kDeployedPosition = 0;

        public static final double kHalfDeployedPosition = 0.264160;
    };

    public static class Feeder {
        public static final int kMotorId1 = 18;
        public static final double kGearRatio = 5;

        public static Slot0Configs slot0() {
            Slot0Configs slot0Configs = new Slot0Configs();
            slot0Configs.kP = 0.005;
            slot0Configs.kS = 0.60;
            return slot0Configs;
        }

        public static final CurrentLimitsConfigs FeederCurrentConfigs = 
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(20)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(20)
                .withSupplyCurrentLowerLimit(20)
                .withSupplyCurrentLowerTime(.75);
    };

    public static class Shooter{
        public static final int kLeaderMotorId = 19;
        public static final int kFollowerMotorId = 20;
        
        public static final CurrentLimitsConfigs ShooterCurrentConfigs = 
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(20)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(20)
                .withSupplyCurrentLowerLimit(20)
                .withSupplyCurrentLowerTime(.75);
        public static final double kGearRatio = 1;

        public static TalonFXConfiguration configs(){
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
        public static final double kGearRatio = 15;
        public static final double spindexerIdleSpeed = 0;
        
        public static Slot0Configs slot0() {
            Slot0Configs slot0Configs = new Slot0Configs();
            slot0Configs.kP = .06;
            slot0Configs.kS = .1;
            return slot0Configs;
        }

        public static final CurrentLimitsConfigs SpindedxerCurrentConfigs = 
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(20)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLimit(20)
                .withSupplyCurrentLowerLimit(20)
                .withSupplyCurrentLowerTime(.75);
    };

    
    public enum RobotType{
        TinmanV2,
        TinmanV1
    }

    public static class Signaling {
        public static double kShiftChangeWarningTime = 5;

        public static double kShift1Start = 130;
        public static double kShift2Start = 105;
        public static double kShift3Start = 80;
        public static double kShift4Start = 55;
        public static double kEndGameStart = 30;

        public static double kShift1StartOffset = kShift1Start + kShiftChangeWarningTime;
        public static double kShift2StartOffset = kShift2Start + kShiftChangeWarningTime;
        public static double kShift3StartOffset = kShift3Start + kShiftChangeWarningTime;
        public static double kShift4StartOffset = kShift4Start + kShiftChangeWarningTime;
    }
}
