// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class Collector extends SubsystemBase {
 private final TalonFX m_collector1 = new TalonFX(Constants.Collector.kMotorId1, TunerConstants.kCANBus);
 private final TalonFX m_collector2 = new TalonFX(Constants.Collector.kMotorId2, TunerConstants.kCANBus);
 private final TalonFX m_collector3 = new TalonFX(Constants.Collector.kMotorId3, TunerConstants.kCANBus);
 private final TalonFX m_collector4 = new TalonFX(Constants.Collector.kMotorId4, TunerConstants.kCANBus); 

 private double collectorSpeed = 2;

  /** Creates a new Collector. */
  public Collector() {
    SmartDashboard.putNumber("Collector Speed", collectorSpeed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.getNumber("Collector Speed", collectorSpeed);

  }
}
