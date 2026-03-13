// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;


public class Signaling extends SubsystemBase {
  /** Creates a new Signaling. */
  private String m_gameData;
  private double m_matchTime;
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;
  private final LEDPattern m_yellow = LEDPattern.solid(Color.kYellow);
  private final LEDPattern m_blue = LEDPattern.solid(Color.kBlue);
  private final LEDPattern m_red = LEDPattern.solid(Color.kRed);
  private final LEDPattern m_black = LEDPattern.solid(Color.kBlack);
  private final CommandXboxController m_DriverController;
  private final Timer m_rumbleTimer = new Timer();
  private boolean m_alerted = false;
  private boolean m_shift1Active = false;
  private Optional <Alliance> m_alliance;

  private enum MaskDirections {
    Up,
    Down
  }
  
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
    m_alliance = DriverStation.getAlliance();
    SmartDashboard.putNumber("Time", m_matchTime);

    if(isHubActive()) {
       if(shouldSetColorMask()) {
          setAlianceColorMask(MaskDirections.Down);
      } else {
        setAlianceColor();
      }
    } else {
      if(shouldSetColorMask()) {
          setAlianceColorMask(MaskDirections.Up);
      } else {
        off();
      }
    }
  }

  private boolean shouldSetColorMask() {
    return (gameTimeBetween(Constants.Signaling.kShift1Start, Constants.Signaling.kShift1StartOffset ) && !m_shift1Active)
        || gameTimeBetween(Constants.Signaling.kShift2Start, Constants.Signaling.kShift2StartOffset)
        || gameTimeBetween(Constants.Signaling.kShift3Start, Constants.Signaling.kShift3StartOffset)
        || gameTimeBetween(Constants.Signaling.kShift4Start, Constants.Signaling.kShift4StartOffset);
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

  private void blue() {
    m_blue.applyTo(m_ledBuffer);
  }

  private void blueMask(MaskDirections maskDirection){
    applyMask(Color.kBlue, maskDirection);
  }

  private void red() {
    m_red.applyTo(m_ledBuffer);
  }

  private void redMask(MaskDirections maskDirection){
    applyMask(Color.kRed, maskDirection);
  }

  private void applyMask(Color color, MaskDirections maskDirection) {
    LEDPattern base = LEDPattern.gradient(GradientType.kContinuous, color);
    
    double warningTime = Constants.Signaling.kShiftChangeWarningTime;
    double progress = (m_matchTime % warningTime) / warningTime;

    if(maskDirection == MaskDirections.Up) {
      progress = 1 - progress;
    }

    double appliedProgress = progress;
    LEDPattern mask = LEDPattern.progressMaskLayer(() -> appliedProgress);

    LEDPattern changing = base.mask(mask);
    changing.applyTo(m_ledBuffer);
  }

  public void yellow(){
    m_yellow.applyTo(m_ledBuffer);
  }

  private void off(){
    m_black.applyTo(m_ledBuffer);
  }

  private void setAlianceColor() {
    if(m_alliance.get() == Alliance.Red) {
      red();
    } else if(m_alliance.get() == Alliance.Blue) {
      blue();
    } else {
      yellow();
    }
  }

  private void setAlianceColorMask(MaskDirections maskDirection) {
    if(m_alliance.get() == Alliance.Red) {
      redMask(maskDirection);
    } else if(m_alliance.get() == Alliance.Blue) {
      blueMask(maskDirection);
    } else {
      yellow();
    }
  }

  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(m_ledBuffer));
  }

  private boolean isHubActive(){
    if(m_alliance.isEmpty()){
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

      m_shift1Active = switch (m_alliance.get()) {
        case Red -> !redInactiveFirst;
        case Blue -> redInactiveFirst;
      };

      if(m_matchTime > Constants.Signaling.kShift1Start) {
        return true;
      } else if (m_matchTime > Constants.Signaling.kShift2Start) {
        return m_shift1Active;
      } else if (m_matchTime > Constants.Signaling.kShift3Start) {
        return !m_shift1Active;
      } else if (m_matchTime > Constants.Signaling.kShift4Start ) {
        return m_shift1Active;
      } else if (m_matchTime > Constants.Signaling.kEndGameStart) {
        return !m_shift1Active;
      } else {
        return true;
      }
  }

  private boolean gameTimeBetween(double minTime, double maxTime){
    return m_matchTime > minTime && m_matchTime < maxTime;
  }

}
