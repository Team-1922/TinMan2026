// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Spindexer extends SubsystemBase {
  TalonFX m_Spindexer = new TalonFX(16, Constants.canbus);
  double m_speed = 1;

  /** Creates a new Spindexer. */
  public Spindexer() {
    SmartDashboard.putNumber("Spindexer Speed", m_speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    m_speed = SmartDashboard.getNumber("Spindexer Speed", m_speed);
  }

  public void loadShooter() {
    m_Spindexer.set(m_speed);
  }

  public void stopShooter() {
    m_Spindexer.set(0);

  }

}
