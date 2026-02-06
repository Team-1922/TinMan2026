// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  TalonFX m_leftShooter = new TalonFX(17, "Drivebase"); // Flywheel
  TalonFX m_rightShooter = new TalonFX(15, "Drivebase"); // HoodWeel

  /** Creates a new Shooter. */
  public Shooter() {
  }

  public void Shoot(double m_tSpeed, double m_bSpeed) {
    m_leftShooter.set(m_tSpeed);
    m_rightShooter.set(-m_bSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
