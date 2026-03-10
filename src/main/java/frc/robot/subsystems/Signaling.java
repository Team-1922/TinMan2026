// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Centimeter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;

import com.ctre.phoenix6.signals.Led1OffColorValue;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class Signaling extends SubsystemBase {
  /** Creates a new Signaling. */
  private String m_gameData;
  private double m_matchTime;
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;
  private LEDPattern m_yellow = LEDPattern.solid(Color.kYellow);
  private LEDPattern m_red = LEDPattern.solid(Color.kRed);
  private final CommandXboxController m_DriverController;
  private final Timer m_rumbleTimer = new Timer();
  private boolean m_alerted = false;
  private final double m_earlyWarningBlink = 3;
  private final double m_earlyWarningProgressBar = 2;
  
  public Signaling(CommandXboxController commandXboxController) {
    m_DriverController = commandXboxController;
    m_led = new AddressableLED(2);
    m_ledBuffer = new AddressableLEDBuffer(66);

    m_led.setLength(m_ledBuffer.getLength());
    m_led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_led.setData(m_ledBuffer);
    m_gameData = DriverStation.getGameSpecificMessage();
    m_matchTime = DriverStation.getMatchTime();

    if(DriverStation.isTeleopEnabled() && isHubActive()){
      if(m_matchTime > 130 - m_earlyWarningBlink && m_matchTime < 130){
        doubleYellowBlink();
        rumble();
      } else if(m_matchTime> 130 - m_earlyWarningProgressBar && m_matchTime < 130){
        yellowProgressBar();
      } else if(m_matchTime > 105 - m_earlyWarningBlink && m_matchTime < 105){
        doubleYellowBlink();
        rumble();
      } else if(m_matchTime> 105 - m_earlyWarningProgressBar && m_matchTime < 105){
        yellowProgressBar();
      } else if(m_matchTime > 55 - m_earlyWarningBlink && m_matchTime < 55){
        doubleYellowBlink();
        rumble();
      } else if(m_matchTime> 55 - m_earlyWarningProgressBar && m_matchTime < 55){
        yellowProgressBar();
      } else{
        yellow();
      }
    } else if(DriverStation.isEnabled() && !isHubActive()){
      
      if(m_matchTime > 80 - m_earlyWarningBlink && m_matchTime < 80){
        redBlink();
      } else if(m_matchTime > 80 - m_earlyWarningBlink && m_matchTime < 80){
        redProgressBar();
      } else if(m_matchTime > 30 - m_earlyWarningBlink && m_matchTime < 30){
        redBlink();
      } else if(m_matchTime > 30 - m_earlyWarningBlink && m_matchTime < 30){
        redProgressBar(); 
      } else {
        red();
      }
      m_alerted = false;
    }
  }

  public void rumble(){  
    if(!m_rumbleTimer.isRunning() && !m_alerted) {
        m_rumbleTimer.restart();
        m_DriverController.setRumble(RumbleType.kBothRumble, 1);
      }
    if(m_rumbleTimer.hasElapsed(1)) {
        m_rumbleTimer.stop();
        m_DriverController.setRumble(RumbleType.kBothRumble, 0);
        m_alerted = true;
      }
  }

  public void yellow() {
    m_yellow.applyTo(m_ledBuffer);
  }

  public void red() {
    m_red.applyTo(m_ledBuffer);
  }

  public void doubleYellowBlink(){
    LEDPattern base = LEDPattern.gradient(GradientType.kContinuous, Color.kYellow);
    LEDPattern pattern = base.blink(Seconds.of(0.5), Seconds.of(0.5));

    pattern.applyTo(m_ledBuffer);
  }

  public void yellowProgressBar(){
    LEDPattern base = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kYellow, Color.kRed);
    LEDPattern mask = LEDPattern.progressMaskLayer(() -> (m_matchTime % 5)/5 );
    LEDPattern somethingThatMakesABitOfSense = base.mask(mask);

    somethingThatMakesABitOfSense.applyTo(m_ledBuffer);
  }

  public void redBlink(){
    LEDPattern base = LEDPattern.gradient(GradientType.kContinuous, Color.kRed);
    LEDPattern pattern = base.blink(Seconds.of(0.5), Seconds.of(0.5));

    pattern.applyTo(m_ledBuffer);
  }

  public void redProgressBar(){
    LEDPattern base = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kRed, Color.kYellow);
    LEDPattern mask = LEDPattern.progressMaskLayer(() -> (m_matchTime % 5)/5 );
    LEDPattern somethingThatMakesABitOfSense = base.mask(mask);

    somethingThatMakesABitOfSense.applyTo(m_ledBuffer);
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

      if(m_matchTime > 130) {
        return true;
      } else if (m_matchTime > 105) {
        return shift1Active;
      } else if (m_matchTime > 80) {
        return !shift1Active;
      } else if (m_matchTime > 55 ) {
        return shift1Active;
      } else if (m_matchTime > 30) {
        return !shift1Active;
      } else {
        return true;
      }
  }

}
