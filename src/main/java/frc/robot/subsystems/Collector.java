// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.generated.TunerConstants;

public class Collector extends SubsystemBase {
  private final TalonFX m_leaderMotor = new TalonFX(
    Constants.Collector.kLeaderID, 
    Constants.superstructureCanbus
  );

  private final TalonFX m_followerMotor = new TalonFX(
    Constants.Collector.kFollowerID, 
    Constants.superstructureCanbus
  );

  private final TalonFX m_positionalMotor = new TalonFX(
    Constants.Collector.kPositinalId, 
    Constants.superstructureCanbus
  );

 private double m_rps = 0;
 CANcoder m_armEncoder = new CANcoder(Constants.Collector.kCANcoderID, Constants.superstructureCanbus);
 private VelocityDutyCycle m_collectorDutyCycle = new VelocityDutyCycle(0).
    withSlot(0);

  /** Creates a new Collector. */
  public Collector() {
      MotorOutputConfigs lfMotorConfig = new MotorOutputConfigs()
      .withInverted(InvertedValue.CounterClockwise_Positive)
      .withNeutralMode(NeutralModeValue.Coast);

       MotorOutputConfigs positinalMotorConfig = new MotorOutputConfigs()
      .withInverted(InvertedValue.CounterClockwise_Positive)
      .withNeutralMode(NeutralModeValue.Brake);
      
      m_leaderMotor.getConfigurator().apply(Constants.Collector.slot0());
      m_leaderMotor.getConfigurator().apply(Constants.Collector.CollectorCurrentConfigs);
      m_leaderMotor.getConfigurator().apply(lfMotorConfig);

      m_followerMotor.getConfigurator().apply(Constants.Collector.slot0());
      m_followerMotor.getConfigurator().apply(Constants.Collector.CollectorCurrentConfigs);
      m_followerMotor.getConfigurator().apply(lfMotorConfig);
      m_followerMotor.setControl(new Follower(
        Constants.Collector.kLeaderID,
        MotorAlignmentValue.Aligned
      ));

      m_positionalMotor.getConfigurator().apply(Constants.Collector.slot0());
      m_positionalMotor.getConfigurator().apply(Constants.Collector.RotatinalCurrentConfigs);
      m_positionalMotor.getConfigurator().apply(positinalMotorConfig);
      m_positionalMotor.setPosition(Constants.Collector.startPos);
      m_positionalMotor.getConfigurator().apply(Constants.Collector.collectorFeedbackConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setTargetRps(double rps) {
    m_rps = rps;
  }

  public void deploy(double angle) {
    m_positionalMotor.
  }
  
  public void collect() {
   if(m_rps > 0) {
      m_leaderMotor.setControl(m_collectorDutyCycle.withVelocity(m_rps * Constants.Collector.kGearRatio));
    }
  }

  public void stopCollector() {
    m_rps = 0;
    m_leaderMotor.stopMotor();
  }
}
