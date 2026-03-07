// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  /** Creates a new LEDs. */
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;
  LEDPattern m_red = LEDPattern.solid(Color.kRed);
 
  public LEDs() {
    m_led = new AddressableLED(0);
    m_ledBuffer = new AddressableLEDBuffer(6);

    m_led.setLength(m_ledBuffer.getLength());
    m_led.start();
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_led.setData(m_ledBuffer);

  }

  public void Red() {
    m_red.applyTo(m_ledBuffer);
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
}
