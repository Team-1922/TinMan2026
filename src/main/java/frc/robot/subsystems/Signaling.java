// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class Signaling extends SubsystemBase {
  /** Creates a new Signaling. */
  private String m_gameData = DriverStation.getGameSpecificMessage();
  double m_matchTime = DriverStation.getMatchTime();
  private final CommandXboxController DriverController = new CommandXboxController(0);
  
  public Signaling() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(m_gameData.length() > 0){
      switch (m_gameData.charAt(0)){
        case 'B' :
          break;
        case 'R' :
          break;
        default :
          break;
      }
    }
  }

  public void startRotatingBack(){
    
      DriverController.setRumble(RumbleType.kBothRumble, 1);
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
