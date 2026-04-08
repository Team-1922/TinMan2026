// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Fahrenheit;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Collector extends SubsystemBase {
  private final TalonFX m_rollerLeaderMotor = new TalonFX(
      Constants.Collector.kRollerLeaderMotorId, 
      Constants.superstructureCanbus
  );

  private final TalonFX m_rollerFollowerMotor = new TalonFX(
      Constants.Collector.kRollerFollowerMotorId,
      Constants.superstructureCanbus
  );

  private final TalonFX m_pivotMotor = new TalonFX(
      Constants.Collector.kPivotMotorId, 
      Constants.superstructureCanbus
  );

  private final CANcoder m_pivotEncoder = new CANcoder(
      Constants.Collector.kPivotCanCoderId,
      Constants.superstructureCanbus
  );
  private final VelocityDutyCycle m_collectorDutyCycle = 
    new VelocityDutyCycle(0)
      .withSlot(0);

  private final PositionDutyCycle m_collectorPostionDutyCycle = 
    new PositionDutyCycle(Constants.Collector.kRetractedPosition);

  /** Creates a new Collector.   */
  public Collector() {
    MotorOutputConfigs rollerLeaderMotorConfig = new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Coast);

    MotorOutputConfigs pivotMotorConfig = new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive);
    
    MotorOutputConfigs rollerFollowerMotorConfig = new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Coast);
    
    m_rollerLeaderMotor.getConfigurator().apply(Constants.Collector.slot0());
    m_rollerLeaderMotor.getConfigurator().apply(
        Constants.Collector.kRollerCurrentConfigs
    );
    m_rollerLeaderMotor.getConfigurator().apply(rollerLeaderMotorConfig);

    m_rollerFollowerMotor.getConfigurator().apply(Constants.Collector.slot0());
    m_rollerFollowerMotor.getConfigurator().apply(
        Constants.Collector.kRollerCurrentConfigs
    );
    m_rollerFollowerMotor.setControl(
        new Follower(
            Constants.Collector.kRollerLeaderMotorId, 
            MotorAlignmentValue.Opposed
        )
    );
   
    m_rollerFollowerMotor.getConfigurator().apply(rollerFollowerMotorConfig);

    m_pivotEncoder.getConfigurator().apply(
        Constants.Collector.kPivotCanCoderConfig
    );

    m_pivotMotor.getConfigurator().apply(Constants.Collector.pivotSlot0());
    m_pivotMotor.getConfigurator().apply(
        Constants.Collector.kPivotCurrentConfigs);
    m_pivotMotor.getConfigurator().apply(pivotMotorConfig);
    m_pivotMotor.getConfigurator().apply(
        Constants.Collector.kPivotFeedbackConfig
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void deploy() {
    pivotCollector(Constants.Collector.kDeployedPosition);}

  public void retract() {
    pivotCollector(Constants.Collector.kRetractedPosition);
  }

  public void halfCollector() {
    pivotCollector(Constants.Collector.kHalfDeployedPosition);
  }

  public void spinCollectorBars() {
    collect(Constants.Collector.krps);
  }

  public void reverseCollectorBars() {
    collect(Constants.Collector.kreverseRps);
  }

  private void pivotCollector(double position) {
    m_pivotMotor.setControl( m_collectorPostionDutyCycle.withPosition(position));
  }
  
  public void collect(double rps) {
    m_rollerLeaderMotor.setControl(
      m_collectorDutyCycle.withVelocity(
        rps * Constants.Collector.kRollerGearRatio
      )
    );
    putDataOnDashboard();
  }

  public void stopCollector() {
    m_rollerLeaderMotor.stopMotor();
  }

  private void putDataOnDashboard() {
    double rollorMotorTemp = m_rollerLeaderMotor.getDeviceTemp().getValue().magnitude();
    double pivotMotorTemp = m_pivotMotor.getDeviceTemp().getValue().magnitude();

    SmartDashboard.putNumber("Motor Temps/Collector/Roller", Celsius.of(rollorMotorTemp).in(Fahrenheit));
    SmartDashboard.putNumber("Motor Temps/Collector/Pivot", Celsius.of(pivotMotorTemp).in(Fahrenheit));
  }
}
