// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.RobotType;
import frc.robot.LimelightHelpers.RawFiducial;

public class Vision extends SubsystemBase {
  /** Creates a new Limelight. */
  public Vision() {
  }

  double dist = 0;
  double tx = 0;
  int id = 0;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("limelight-front");
    for (RawFiducial fiducial : fiducials) {
      id = fiducial.id;
      tx = LimelightHelpers.getTX("limelight-front");
      dist = fiducial.distToCamera;
      
      SmartDashboard.putNumberArray(
        "distToCamera", new Double[] { (double) id, dist });
    }
  }

  public double getDist() {
    return Constants.robotType == RobotType.TinmanV0 ? dist : Constants.targetDistanceToHub;
  }

  public double getTx() {
    return tx;
  }

  public double getId() {
    return id;
  }
}
