// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class Signaling extends SubsystemBase {
  /** Creates a new Signaling. */
  private final CommandXboxController DriverController = new CommandXboxController(0);
  public Signaling() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void isTheBotAlined(boolean aligned){
    if(aligned) {
      DriverController.setRumble(RumbleType.kBothRumble, 1);
    }
  }
}
