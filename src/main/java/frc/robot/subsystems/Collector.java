// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.generated.TunerConstants;

public class Collector extends SubsystemBase {
 private final TalonFX m_collector1 = new TalonFX(
  Constants.Collector.kMotorId1, 
  Constants.superstructureCanbus);

 private double m_rps = 0;
 private VelocityDutyCycle m_collectorDutyCycle = new VelocityDutyCycle(0).
    withSlot(0);

  /** Creates a new Collector. */
  public Collector() {
    if(Constants.robotType == RobotType.TinmanV1) {
      MotorOutputConfigs motorConfig = new MotorOutputConfigs()
      .withInverted(InvertedValue.Clockwise_Positive)
      .withNeutralMode(NeutralModeValue.Coast);
      m_collector1.getConfigurator().apply(Constants.Collector.slot0());
      m_collector1.getConfigurator().apply(Constants.Collector.CollectorCurrentConfigs);
      m_collector1.getConfigurator().apply(motorConfig);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setTargetRps(double rps) {
    m_rps = rps;
  }
  
  public void collect() {
   if(m_rps > 0) {
      m_collector1.setControl(m_collectorDutyCycle.withVelocity(m_rps * Constants.Collector.kGearRatio));
    }
  }

  public void stopCollector() {
    m_rps = 0;
    m_collector1.stopMotor();
  }
}
