// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.BangBangController;

public class Shooter extends SubsystemBase {
  private final TalonFX m_leaderMotor = new TalonFX(
      Constants.Shooter.kLeaderMotorId,
      Constants.superstructureCanbus
  );

  private final TalonFX m_followerMotor = new TalonFX(
      Constants.Shooter.kFollowerMotorId,
      Constants.superstructureCanbus
  );

 BangBangController m_controller = new BangBangController();
 private double m_rps = 0;


  /** Creates a new Shooter. */
  public Shooter() {
    MotorOutputConfigs motorConfig = new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Coast);

    m_leaderMotor.getConfigurator().apply(motorConfig);
    m_followerMotor.getConfigurator().apply(motorConfig);

    m_followerMotor.setControl(new Follower(
        Constants.Shooter.kLeaderMotorId,
        MotorAlignmentValue.Aligned
    ));
  }

  public double getVelocity(){
    return m_leaderMotor.getVelocity().getValueAsDouble();
  }

  public void setTargetRps(double rps) {
    m_rps = rps;
  }

  public void stop(){
    m_rps = 0;
    m_leaderMotor.stopMotor();
  }

  @Override
  public void periodic() {
    if(m_rps > 0){
      m_leaderMotor.set(
          m_controller.calculate(
              m_leaderMotor.getVelocity().getValueAsDouble(), 
              m_rps
          )
      );
    }
  }
}
