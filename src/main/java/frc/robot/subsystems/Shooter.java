// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import edu.wpi.first.math.controller.BangBangController;

public class Shooter extends SubsystemBase {
 private final TalonFX m_shooter1 = new TalonFX(Constants.Shooter.kMotorId1, TunerConstants.kCANBus);
 private final TalonFX m_shooter2 = new TalonFX(Constants.Shooter.kMotorId2, TunerConstants.kCANBus);
 BangBangController controller = new BangBangController();


  /** Creates a new Shooter. */
  public Shooter() {
    m_shooter1.setNeutralMode(NeutralModeValue.Coast);
    m_shooter2.setNeutralMode(NeutralModeValue.Coast);
    m_shooter1.setControl(new Follower(Constants.Shooter.kMotorId1, MotorAlignmentValue.Aligned));
  }

  public void Shoot(double rpm) {
    m_shooter1.set(controller.calculate(m_shooter1.getVelocity().getValueAsDouble(), rpm));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
