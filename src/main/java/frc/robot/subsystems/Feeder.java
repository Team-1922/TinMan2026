// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class Feeder extends SubsystemBase {
  /** Creates a new Feeder. */
 private final TalonFX m_Feeder = new TalonFX(Constants.Feeder.kMotorId1, TunerConstants.kCANBus);
 BangBangController m_controller = new BangBangController();
 private double m_rps = 0;

  public Feeder() {
    MotorOutputConfigs motorConfig = new MotorOutputConfigs()
    .withInverted(InvertedValue.CounterClockwise_Positive)
    .withNeutralMode(NeutralModeValue.Coast);
    
    SmartDashboard.putNumber("Feeder RPS", m_rps);
    m_Feeder.getConfigurator().apply(motorConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     if(m_rps > 0) {
        m_rps = SmartDashboard.getNumber("Feeder RPS", m_rps);
        m_Feeder.set(m_controller.calculate(m_Feeder.getVelocity().getValueAsDouble(), m_rps));
     }
  }

  public void setTargetRps(double rps) {
    m_rps = rps;
  }

  public void stop() {
    m_rps = 0;
    m_Feeder.stopMotor();
  }
}
