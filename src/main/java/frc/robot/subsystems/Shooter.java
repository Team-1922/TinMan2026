// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  CANBus m_canbus;
  TalonFX m_shooter1 = new TalonFX(Constants.shooterMotorID1, m_canbus);
  TalonFX m_shooter2 = new TalonFX(Constants.shooterMotorID2, m_canbus);

  /** Creates a new Shooter. */
  public Shooter( CANBus canbus) {
    m_canbus = canbus;
  }

  public void Shoot(double m_s1Speed, double m_s2Speed) {
    m_shooter1.set(m_s1Speed);
    m_shooter2.set(-m_s2Speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
