// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Collector extends SubsystemBase {
  private final TalonFX m_rollerMotor = new TalonFX(
    Constants.Collector.kRollerMotorId, 
    Constants.superstructureCanbus
  );

  private final TalonFX m_pivotMotor = new TalonFX(
    Constants.Collector.kPivotMotorId, 
    Constants.superstructureCanbus
  );

 private double m_rps = 0;
 private final CANcoder m_pivotEncoder = new CANcoder(Constants.Collector.kPivotCanCoderId, Constants.superstructureCanbus);
 private final VelocityDutyCycle m_collectorDutyCycle = new VelocityDutyCycle(0).
    withSlot(0);

  /** Creates a new Collector. */
  public Collector() {
    MotorOutputConfigs rollerMotorConfig = new MotorOutputConfigs()
    .withInverted(InvertedValue.CounterClockwise_Positive)
    .withNeutralMode(NeutralModeValue.Coast);

      MotorOutputConfigs pivotMotorConfig = new MotorOutputConfigs()
    .withInverted(InvertedValue.Clockwise_Positive)
    .withNeutralMode(NeutralModeValue.Brake);
    
    m_rollerMotor.getConfigurator().apply(Constants.Collector.slot0());
    m_rollerMotor.getConfigurator().apply(Constants.Collector.kRollerCurrentConfigs);
    m_rollerMotor.getConfigurator().apply(rollerMotorConfig);

    m_pivotEncoder.getConfigurator().apply(Constants.Collector.kPivotCanCoderConfig);

    m_pivotMotor.getConfigurator().apply(Constants.Collector.slot0());
    m_pivotMotor.getConfigurator().apply(Constants.Collector.kPivotCurrentConfigs);
    m_pivotMotor.getConfigurator().apply(pivotMotorConfig);
    m_pivotMotor.getConfigurator().apply(Constants.Collector.kPivotFeedbackConfig);
    m_pivotMotor.getConfigurator().apply(Constants.Collector.kPivotMotionMagicConfigs);

    m_pivotMotor.setPosition(Constants.Collector.startPos);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setTargetRps(double rps) {
    m_rps = rps;
  }

  public void deploy() {
    m_pivotMotor.setControl(Constants.Collector.kRequest.withPosition(Constants.Collector.endPos));
  }

  public void retract() {
    m_pivotMotor.setControl(Constants.Collector.kRequest.withPosition(0));
  }
  
  public void collect() {
   if(m_rps > 0) {
      m_rollerMotor.setControl(m_collectorDutyCycle.withVelocity(m_rps * Constants.Collector.kGearRatio));
    }
  }

  public void stopCollector() {
    m_rps = 0;
    m_rollerMotor.stopMotor();
  }
}
