// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.util.Optional;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class Signaling extends SubsystemBase {
  /** Creates a new Signaling. */
  private String m_gameData = DriverStation.getGameSpecificMessage();
  double m_matchTime = DriverStation.getMatchTime();
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;
  LEDPattern m_yellow = LEDPattern.solid(Color.kYellow);
  private final CommandXboxController DriverController = new CommandXboxController(0);
  
  public Signaling() {
     m_led = new AddressableLED(2);
    m_ledBuffer = new AddressableLEDBuffer(66);

    m_led.setLength(m_ledBuffer.getLength());
    m_led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_led.setData(m_ledBuffer);
  }

  public void startRotatingBack(){
    
      DriverController.setRumble(RumbleType.kBothRumble, 1);
  }

   public void yellow() {
    m_yellow.applyTo(m_ledBuffer);
  }

  public void yellowScroll(){
    Distance ledSpacing = Meters.of(1 / 120.0);
    LEDPattern base = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kYellow, Color.kBlack);
    LEDPattern pattern = base.scrollAtRelativeSpeed(Percent.per(Second).of(25));
    LEDPattern absolute = base.scrollAtAbsoluteSpeed(Centimeter.per(Second).of(12.5), ledSpacing);

    pattern.applyTo(m_ledBuffer);
  }

   public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(m_ledBuffer));
  }

  private boolean isHubActive(){
    Optional <Alliance> alliance = DriverStation.getAlliance();
    if(alliance.isEmpty()){
      return false;
    }

    if(DriverStation.isAutonomousEnabled()){
      return true;
    }

    if(!DriverStation.isTeleopEnabled()){
      return false;
    }

    if(m_gameData.isEmpty()){
      return true;
    }

    boolean redInactiveFirst = false;
      switch (m_gameData.charAt(0)){
        case 'R' -> redInactiveFirst = true;
        case 'B' -> redInactiveFirst = false;
        default -> {
          return true;
        }
      }

      boolean shift1Active = switch (alliance.get()) {
        case Red -> !redInactiveFirst;
        case Blue -> redInactiveFirst;
      };

      if(m_matchTime > 130 ) {
        return true;
      } else if (m_matchTime > 105) {
        return shift1Active;
      } else if (m_matchTime > 80) {
        return !shift1Active;
      } else if (m_matchTime > 55) {
        return shift1Active;
      } else if (m_matchTime > 30) {
        return !shift1Active;
      } else {
        return true;
      }
  }

}
