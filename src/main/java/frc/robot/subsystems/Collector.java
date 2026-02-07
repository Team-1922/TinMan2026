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
  TalonFX m_collector1 = new TalonFX(Constants.collectorID1, TunerConstants.kCANBus);
  TalonFX m_collector2 = new TalonFX(Constants.collectorID2, TunerConstants.kCANBus);
  TalonFX m_collector3 = new TalonFX(Constants.collectorID3, TunerConstants.kCANBus);
  TalonFX m_collector4 = new TalonFX(Constants.collectorID4, TunerConstants.kCANBus); 

  double collectorSpeed = 2;

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
