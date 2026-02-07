// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
  /** Creates a new Feeder. */
  CANBus m_canbus;
  TalonFX m_Feeder = new TalonFX(Constants.feederID, m_canbus);
  double m_feedSpeed = .2;

  public Feeder(CANBus canbus) {
    SmartDashboard.putNumber("Load Shooter", m_feedSpeed);
    m_canbus = canbus;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_feedSpeed = SmartDashboard.getNumber("Load Shooter", m_feedSpeed);
  }

  public void feed() {
    m_Feeder.set(m_feedSpeed);
  }

  public void stopFeed() {
    m_Feeder.set(0);
  }
}
