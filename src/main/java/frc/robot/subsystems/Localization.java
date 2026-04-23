// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

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
  //all of these distances are in meters
  private final CommandSwerveDrivetrain m_drivetrain;
  private Field2d m_Field2d = new Field2d();
  private boolean m_hasTarget = false;
  private double m_shooterX;
  private double m_shooterY;
  private double m_deltaX;
  private double m_deltaY;
  private double m_targetYaw;
  private double m_errorYaw;
  private double m_errorX;
  private double m_errorY;
  private double m_shooterXRobotFrame = -0.2159;
  private double m_shooterYRobotFrame = 0.19685;
  private Pose2d m_targetPose = new Pose2d();
  private Pose2d m_initialRobotPose = new Pose2d();
  private final double m_hubY = 4.035;
  private final double m_blueHubX = 4.625594;
  private final double m_redHubX = 11.915394;
  private final double m_shuttleYOffset = 1.6256; 
  private final double m_shuttleXOffset = 1.0668; 
  
  /** Creates a new Localization. */
  public Localization(CommandSwerveDrivetrain drivetrain) {
    m_drivetrain = drivetrain;
    m_drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
  };

  public void setTarget(){
    if(m_shooterX < m_blueHubX) {
      if(DriverStation.getAlliance().get() == Alliance.Blue){
        SmartDashboard.putString("target", "blue_hub");
        m_targetPose = new Pose2d(m_blueHubX, m_hubY,null);
        m_hasTarget = true;
      } else {
        SmartDashboard.putString("target", "none");
        m_hasTarget = false;
      }
    } else if(m_shooterX > m_redHubX) {
      if(DriverStation.getAlliance().get() == Alliance.Red){
        m_targetPose = new Pose2d(m_redHubX, m_hubY, null);
        m_hasTarget = true;
      } else {
        m_hasTarget = false;
      }
    } else if(m_shooterY > m_hubY) {
      if(DriverStation.getAlliance().get() == Alliance.Blue){
        m_targetPose = new Pose2d(m_blueHubX - m_shuttleXOffset, m_hubY + m_shuttleYOffset,null);
        m_hasTarget = true;
      } else {
        m_targetPose = new Pose2d(m_redHubX + m_shuttleXOffset, m_hubY + m_shuttleYOffset,null);
        m_hasTarget = true;
      }
    } else {
      if(DriverStation.getAlliance().get() == Alliance.Blue){
        m_targetPose = new Pose2d(m_blueHubX - m_shuttleXOffset, m_hubY - m_shuttleYOffset,null);
        m_hasTarget = true;
      } else {
        m_targetPose = new Pose2d(m_redHubX + m_shuttleXOffset, m_hubY - m_shuttleYOffset,null);
        m_hasTarget = true;
      }
    }
  }
 
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("hasTarget", m_hasTarget);
    // This method will be called once per scheduler run
    m_initialRobotPose = m_drivetrain.getPose();
    double yaw = m_initialRobotPose.getRotation().getDegrees();
    
    processLimelight(Constants.frontLimelightName, yaw);
    
    if(Constants.useRightLimelight) {
      processLimelight(Constants.rightLimelightName, yaw);
    }

    Pose2d updatedRobotPose = m_drivetrain.getPose();
    m_Field2d.setRobotPose(updatedRobotPose);
    SmartDashboard.putData("Field2d", m_Field2d);

    double updatedYaw = updatedRobotPose.getRotation().getRadians();
    m_shooterX = updatedRobotPose.getX() 
      + Math.cos(updatedYaw) * m_shooterXRobotFrame
      - Math.sin(updatedYaw) * m_shooterYRobotFrame;
      
    m_shooterY = updatedRobotPose.getY() 
      + Math.cos(updatedYaw) * m_shooterYRobotFrame 
      + Math.sin(updatedYaw) * m_shooterXRobotFrame;

    double timeOfFlight = distFromTarget() / 2;

    double xOffset = m_drivetrain.getFieldRelativeSpeeds().vxMetersPerSecond * timeOfFlight;
    double yOffset = m_drivetrain.getFieldRelativeSpeeds().vyMetersPerSecond * timeOfFlight;
    
    m_deltaX = m_targetPose.getX() - m_shooterX - xOffset;
    m_deltaY = m_targetPose.getY() - m_shooterY - yOffset;
    m_targetYaw = Math.atan2(m_deltaY, m_deltaX);
    m_errorYaw = MathUtil.angleModulus(m_targetYaw - updatedYaw);
    m_errorX = 
      m_deltaX - Constants.maxTargetDistanceToTarget * Math.cos(m_targetYaw);
    m_errorY = 
      m_deltaY - Constants.maxTargetDistanceToTarget * Math.sin(m_targetYaw);
    /* 
    SmartDashboard.putNumber("current_yaw", updatedYaw);
    SmartDashboard.putNumber("target_yaw", m_targetYaw);
    SmartDashboard.putNumber("error_x", m_errorX);
    SmartDashboard.putNumber("error_y", m_errorY);
    SmartDashboard.putNumber("error_yaw", m_errorYaw);
    */
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

  public double distFromTarget() {
    return Math.sqrt(m_deltaX * m_deltaX + m_deltaY * m_deltaY);
  }

  public boolean hasTarget(){
   return m_hasTarget;
  }

  private void processLimelight(String limelightName, double yaw) {
    boolean usedLimelight = false;
    LimelightHelpers.SetRobotOrientation(limelightName, yaw, 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate poseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
    if(!shouldReject(poseEstimate)) {
      m_drivetrain.addVisionMeasurement(
        poseEstimate.pose,
        poseEstimate.timestampSeconds
      );

      usedLimelight = true;
    }

    //SmartDashboard.putBoolean("Using" + limelightName + " vision", usedLimelight);
  }

  private boolean shouldReject(LimelightHelpers.PoseEstimate poseEstimate) {
    // if our angular velocity is greater than 360 degrees per second or if the limelight can't see any tags, ignore vision updates
    return Math.abs(
      m_drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()
    ) > 720
      || poseEstimate == null
      || poseEstimate.tagCount <= 1
      || Math.pow(
           m_Field2d.getRobotPose().getX() - m_initialRobotPose.getX(),
           2
         ) +
         Math.pow(
           m_Field2d.getRobotPose().getY() - m_initialRobotPose.getY(),
           2
         ) > 1 ;//If the vision believes we are more than 1m from where the odometry thinks we are we should assume that it's wrong
  }
}
