// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.Pigeon2;


import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.TunerConstants;

public class Localization extends SubsystemBase {
  private final CommandSwerveDrivetrain m_drivetrain;
  Field2d m_Field2d = new Field2d();
  /** Creates a new Localization. */
  public Localization(CommandSwerveDrivetrain drivetrain) {
    m_drivetrain = drivetrain;
  };
 
  public Pose2d getPose2dEstimate(){
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
  }
}
