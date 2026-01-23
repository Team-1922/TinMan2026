// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Spindexer extends SubsystemBase {
  public TalonFX m_Spindexer = new TalonFX(3);
  public double speed = .2;
  /** Creates a new Spindexer. */
  public Spindexer() {
    SmartDashboard.putNumber("Spindexer Speed", speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    speed = SmartDashboard.getNumber("Spindexer Speed", speed);
    SmartDashboard.putNumber("Speed", speed);
  }

  public void loadShooter(){
    m_Spindexer.set(speed);
  }

}
