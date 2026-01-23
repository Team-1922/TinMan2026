// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Collector extends SubsystemBase {
  public TalonFX m_collector1 = new TalonFX(5);
  public TalonFX m_collector2 = new TalonFX(6);
  public TalonFX m_collector3 = new TalonFX(7);
  public TalonFX m_collector4 = new TalonFX(8);

  public double justSpin = 2;

  /** Creates a new Collector. */
  public Collector() {
    SmartDashboard.putNumber("Just Spin", justSpin);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.getNumber("Just Spin", justSpin);
    
  }
}
