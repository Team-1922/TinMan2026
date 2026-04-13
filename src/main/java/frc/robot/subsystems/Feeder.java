// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Fahrenheit;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
  /** Creates a new Feeder. */
  private final TalonFX m_Feeder = new TalonFX(
    Constants.Feeder.kMotorId1, 
    Constants.superstructureCanbus
  );
  private VelocityDutyCycle m_feederDutyCycle = new VelocityDutyCycle(0)
    .withSlot(0);

  private double m_rps = 0;

  public Feeder() {
    MotorOutputConfigs motorConfig = new MotorOutputConfigs()
    .withInverted(InvertedValue.Clockwise_Positive)
    .withNeutralMode(NeutralModeValue.Coast);
    
    m_Feeder.getConfigurator().apply(motorConfig);
    m_Feeder.getConfigurator().apply(Constants.Feeder.FeederCurrentConfigs);
    m_Feeder.getConfigurator().apply(Constants.Feeder.slot0());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    putDataOnDashboard();
  }

  public void setTargetRps(double rps) {
    rps = rps * Constants.Feeder.kGearRatio;
    if(m_rps != rps){
      m_rps = rps;
      m_Feeder.setControl(
        m_feederDutyCycle.withVelocity(
          m_rps
        )
      );
    }
  }

  public void stop() {
    m_Feeder.stopMotor();
    m_rps = 0;
  }

  public double getVelocity(){
    return m_Feeder.getVelocity().getValueAsDouble();
  }

  private void putDataOnDashboard() {
    double motorTemp = m_Feeder.getDeviceTemp().getValue().magnitude();

    SmartDashboard.putNumber("Motor Temps/Feeder", Celsius.of(motorTemp).in(Fahrenheit));
    SmartDashboard.putNumber("Feeder Motor RPS", m_Feeder.getVelocity().getValueAsDouble() * Constants.Feeder.kGearRatio);
  }
}
