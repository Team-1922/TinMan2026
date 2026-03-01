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
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

import edu.wpi.first.math.util.Units;
import frc.robot.generated.TunerConstants;


/** Add your docs here. */
public final class Constants {

    public static final RobotType robotType = RobotType.TinmanV2;

    public static final double autoAlignDistanceThreshold = 
        Meters.of(.1).in(Meters);
    public static final double targetDistanceToHub = Meters.of(2.7).in(Meters);
    public static String middleLimeLight = "limelight-front";
    public static String drivebaseCanbusName = "Drivebase";
    public static final CANBus superstructureCanbus =  switch(Constants.robotType) {
            case TinmanV2 -> TunerConstants.kCANBus;
            case TinmanV1 ->  CANBus.roboRIO();
        };

    public static class Collector {
        public static final int kLeaderID = 14;
        public static final int kFollowerID = 15;
        public static final int kPositinalId = 16;
        public static final int kMotorId4 = 17;
        public static final double kGearRatio = 2;
        public static final int kCANcoderID = 22;

        public static Slot0Configs slot0() {
            Slot0Configs slot0Configs = new Slot0Configs();
            slot0Configs.kP = .06;
            slot0Configs.kS = .1;
            return slot0Configs;
        }

        public static final CurrentLimitsConfigs CollectorCurrentConfigs = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(20)
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(20)
        .withSupplyCurrentLowerLimit(20)
        .withSupplyCurrentLowerTime(.75);

        public static final CurrentLimitsConfigs RotatinalCurrentConfigs = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(20)
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLimit(20)
        .withSupplyCurrentLowerLimit(20)
        .withSupplyCurrentLowerTime(.75);

        public static final FeedbackConfigs collectorFeedbackConfig = new FeedbackConfigs()
        .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
        .withFeedbackRemoteSensorID(kCANcoderID)
        .withRotorToSensorRatio(25)
        .withSensorToMechanismRatio(1);

        public static final CANcoderConfiguration colectorCANcoderConfig = new CANcoderConfiguration().withMagnetSensor(
                new MagnetSensorConfigs()
                .withMagnetOffset(-0.477783203125)
                .withAbsoluteSensorDiscontinuityPoint(.5)
        );

        public static final MotionMagicConfigs CollectorMotionMagicConfigs = new MotionMagicConfigs()
        .withMotionMagicExpo_kV(2)
        .withMotionMagicExpo_kA(2);

        public static final double endPos = Units.degreesToRotations(90);
        
        public static final double startPos = 0; 
        public static final MotionMagicExpoDutyCycle kRequest = new MotionMagicExpoDutyCycle(0);
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

        public static final CurrentLimitsConfigs FeederCurrentConfigs = new CurrentLimitsConfigs()
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
        
        public static final CurrentLimitsConfigs ShooterCurrentConfigs = new CurrentLimitsConfigs()
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
        public static final double kGearRatio = 6;
        public static final double spindexerIdleSpeed = 0;
        
        public static Slot0Configs slot0() {
            Slot0Configs slot0Configs = new Slot0Configs();
            slot0Configs.kP = .06;
            slot0Configs.kS = .1;
            return slot0Configs;
        }

        public static final CurrentLimitsConfigs SpindedxerCurrentConfigs = new CurrentLimitsConfigs()
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
}
