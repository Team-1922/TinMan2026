// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VelocityDutyCycle;

public class Spindexer extends SubsystemBase {
  private final TalonFX m_Spindexer = new TalonFX(
    Constants.Spindexer.kMotorId1,
    Constants.drivebaseCanbusName
  );
  private double m_rps = 0;
  private VelocityDutyCycle m_spindexerDutyCycle = new VelocityDutyCycle(0)
    .withSlot(0);

  /** Creates a new Spindexer. */
  public Spindexer() {
     MotorOutputConfigs motorConfig = new MotorOutputConfigs()
      .withInverted(InvertedValue.Clockwise_Positive)
      .withNeutralMode(NeutralModeValue.Coast);
    m_Spindexer.getConfigurator().apply(motorConfig);
    m_Spindexer.getConfigurator().apply(Constants.Spindexer.slot0());
  }

  
  public void setTargetRps(double rps) {
    m_rps = rps;
  }

  public void setIdleSpeed() {
    m_rps = Constants.Spindexer.spindexerIdleSpeed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run   
    if(m_rps != 0) {
      m_Spindexer.setControl(
        m_spindexerDutyCycle.withVelocity(
          m_rps * Constants.Spindexer.kGearRatio
        )
      );
    }
    else if(m_Spindexer.getVelocity().getValueAsDouble() != 0){
      m_Spindexer.stopMotor();
    }
    SmartDashboard.putNumber("Spindexer Motor RPS", m_rps * Constants.Spindexer.kGearRatio);
  }
}
