// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase {
  TalonFX m_Spindexer = new TalonFX(3);
  TalonFX m_loadShooter = new TalonFX(4);
  double m_speed = .2;
  public double m_feedSpeed = .2;
  /** Creates a new Spindexer. */
  public Spindexer() {
    SmartDashboard.putNumber("Spindexer Speed", m_speed);
    SmartDashboard.putNumber("Load Shooter", m_feedSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    m_speed = SmartDashboard.getNumber("Spindexer Speed", m_speed);
    m_feedSpeed = SmartDashboard.getNumber("Load Shooter", m_feedSpeed);
  }

  public void loadShooter(){
    m_Spindexer.set(m_speed);
    m_loadShooter.set(m_feedSpeed);
  }

  public void stopShooter(){
    m_Spindexer.set(0);
    m_loadShooter.set(0);
  }

}
