// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class Collector extends SubsystemBase {
 private final TalonFX m_collector1 = new TalonFX(Constants.Collector.kMotorId1, TunerConstants.kCANBus);
 private double m_rps = 0;

  /** Creates a new Collector. */
  public Collector() {
    SmartDashboard.putNumber(getName(), m_rps);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.getNumber(getName(), m_rps);
  }

  public double setTargetRps(double rps) {
    m_rps = rps;
    return rps;
  }

  public void stopCollector() {
    m_rps = 0;
  }

  public void collect() {
    m_collector1.setControl( new VelocityDutyCycle(m_rps));
  }
}
