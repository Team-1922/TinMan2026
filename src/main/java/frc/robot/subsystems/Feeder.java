// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Feeder extends SubsystemBase {
  /** Creates a new Feeder. */
 private final TalonFX m_Feeder = new TalonFX(
      Constants.Feeder.kMotorId1, 
      Constants.drivebaseCanbusName
  );
 private double m_rps = 0;
 private VelocityDutyCycle m_feederDutyCycle = new VelocityDutyCycle(0)
      .withSlot(0);

  public Feeder() {
    MotorOutputConfigs motorConfig = new MotorOutputConfigs()
      .withInverted(InvertedValue.CounterClockwise_Positive)
      .withNeutralMode(NeutralModeValue.Coast);
    
    m_Feeder.getConfigurator().apply(motorConfig);
    m_Feeder.getConfigurator().apply(Constants.Feeder.slot0());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(m_rps > 0) {    
      m_Feeder.setControl(
          m_feederDutyCycle.withVelocity(
              m_rps * Constants.Feeder.kGearRatio
          )
      );
    }
    SmartDashboard.putNumber("Feeder Motor RPS", m_rps * Constants.Feeder.kGearRatio);
  }

  public void setTargetRps(double rps) {
    m_rps = rps;
  }

  public void stop() {
    m_rps = 0;
    m_Feeder.stopMotor();
  }

  public double getSpeed(){
    return m_Feeder.getVelocity().getValueAsDouble();
  }
}
