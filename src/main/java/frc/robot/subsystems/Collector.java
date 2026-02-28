// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.generated.TunerConstants;

public class Collector extends SubsystemBase {
 private final TalonFX m_leaderMotor = new TalonFX(
  Constants.Collector.kMotorId1, 
  Constants.superstructureCanbus);

  private final TalonFX m_followerMotor = new TalonFX(
  Constants.Collector.kMotorId2, 
  Constants.superstructureCanbus);

   private final TalonFX m_positionalMotor = new TalonFX(
  Constants.Collector.kMotorId3, 
  Constants.superstructureCanbus);

 private double m_rps = 0;
 private VelocityDutyCycle m_collectorDutyCycle = new VelocityDutyCycle(0).
    withSlot(0);

  /** Creates a new Collector. */
  public Collector() {
      MotorOutputConfigs motorConfig = new MotorOutputConfigs()
      .withInverted(InvertedValue.CounterClockwise_Positive)
      .withNeutralMode(NeutralModeValue.Coast);
      
      m_leaderMotor.getConfigurator().apply(Constants.Collector.slot0());
      m_leaderMotor.getConfigurator().apply(Constants.Collector.CollectorCurrentConfigs);
      m_leaderMotor.getConfigurator().apply(motorConfig);

      m_followerMotor.getConfigurator().apply(Constants.Collector.slot0());
      m_followerMotor.getConfigurator().apply(Constants.Collector.CollectorCurrentConfigs);
      m_followerMotor.getConfigurator().apply(motorConfig);
      m_followerMotor.setControl(new Follower(
        Constants.Collector.kMotorId1,
        MotorAlignmentValue.Aligned
      ));

      m_positionalMotor.getConfigurator().apply(Constants.Collector.slot0());
      m_positionalMotor.getConfigurator().apply(Constants.Collector.RotatinalCurrentConfigs);
      m_positionalMotor.getConfigurator().apply(motorConfig);
      m_positionalMotor.setPosition(Constants.Collector.startPose);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setTargetRps(double rps) {
    m_rps = rps;
  }

  public void deploy(double angle) {
    m_positionalMotor.setPosition(angle);
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
