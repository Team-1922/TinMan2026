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

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private final TalonFX m_climber1 = new TalonFX(
      Constants.Climber.kMotorId1,
      Constants.superstructureCanbus);

  private double m_rps = 0;
  private VelocityDutyCycle m_climberDutyCycle = new VelocityDutyCycle(0).withSlot(0);
  
  public Climber() {
    MotorOutputConfigs motorConfig = new MotorOutputConfigs()
      .withInverted(InvertedValue.Clockwise_Positive)
      .withNeutralMode(NeutralModeValue.Brake);
    m_climber1.getConfigurator().apply(Constants.Climber.slot0());
    m_climber1.getConfigurator().apply(motorConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double setTargetRps(double rps) {
    m_rps = rps;
    return m_rps;
  }

  public void climb() {
    if (m_rps >= 0) {
      m_climber1.setControl(m_climberDutyCycle.withVelocity(m_rps * Constants.Climber.kGearRatio));
    }
  }

  public void stopClimber(){
    if (m_rps <= 0) {
      m_climber1.setControl(m_climberDutyCycle.withVelocity(0));
    }
  }
}
