// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Localization extends SubsystemBase {
  private final CommandSwerveDrivetrain m_drivetrain;
  Field2d m_Field2d = new Field2d();
  double m_deltaX;
  double m_deltaY;
  double m_targetYaw;
  double m_errorYaw;
  double m_errorX;
  double m_errorY;
  private final Pose2d m_hubpose;
  final Pose2d blueHubPose2d = new Pose2d(5.22, 4.035, null);
  final Pose2d redHubPose2d = new Pose2d(11.32, 4.035, null);
  /** Creates a new Localization. */
  public Localization(CommandSwerveDrivetrain drivetrain) {
    m_drivetrain = drivetrain;
    
    m_hubpose = DriverStation.getAlliance().get() == Alliance.Blue 
    ? blueHubPose2d 
    : redHubPose2d;
  };
 
  public Pose2d getPose2dEstimate() {
    return m_drivetrain.getPose();
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    LimelightHelpers.SetRobotOrientation("limelight-front", getPose2dEstimate().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2_estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
    Boolean doRejectUpdate = false;
    // if our angular velocity is greater than 360 degrees per second or if the limelight can't see any tags, ignore vision updates
    if(Math.abs(m_drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720 || mt2_estimate.tagCount == 0)
    {
      doRejectUpdate = true;
    }
    if(!doRejectUpdate)
    {
      m_drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
      m_drivetrain.addVisionMeasurement(
        mt2_estimate.pose,
        mt2_estimate.timestampSeconds);
    }
    m_Field2d.setRobotPose(getPose2dEstimate());
    SmartDashboard.putData("Field2d", m_Field2d);

    Pose2d robotPose = m_drivetrain.getPose();
    m_deltaX = m_hubpose.getX() - robotPose.getX();
    m_deltaY = m_hubpose.getY() - robotPose.getY();
    m_targetYaw = Math.atan2(m_deltaY, m_deltaX);
    m_errorYaw = 
      MathUtil.angleModulus(m_targetYaw - robotPose.getRotation().getRadians());
    m_errorX = m_deltaX - Constants.targetDistanceToHub * Math.cos(m_targetYaw);
    m_errorY = m_deltaY - Constants.targetDistanceToHub * Math.sin(m_targetYaw);
    
    SmartDashboard.putNumber("target_yaw", m_targetYaw);
    SmartDashboard.putNumber("error_x", m_errorX);
    SmartDashboard.putNumber("error_y", m_errorY);
    SmartDashboard.putNumber("error_yaw", m_errorYaw);
  }

  public double getM_errorX() {
    return m_errorX;
  }

  public double getM_errorY() {
    return m_errorY;
  }

  public double getM_errorYaw() {
    return m_errorYaw;
  }

  public double distFromHub() {
    return Math.sqrt(m_deltaX * m_deltaX + m_deltaY * m_deltaY);
  }
  
}
