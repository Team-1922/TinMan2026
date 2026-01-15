package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class VisionConstants {
    private VisionConstants() {} // Prevent instantiation
    
    public static final String LIMELIGHT_NAME = "limelight";
    
    // Camera mounting
    public static final double CAMERA_HEIGHT_METERS = 0.5;
    public static final double CAMERA_PITCH_DEGREES = -20.0;
    
    public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
        new Translation3d(0.0, 0.0, CAMERA_HEIGHT_METERS),
        new Rotation3d(0, Math.toRadians(CAMERA_PITCH_DEGREES), 0)
    );
    
    // Vision parameters
    public static final double DETECTION_RANGE_METERS = 4.5;
    public static final double DEFAULT_XY_STD_DEV = 0.5;
    public static final double DEFAULT_THETA_STD_DEV_DEGREES = 6.0;
    
    // Field layout
    public static final AprilTagFieldLayout FIELD_LAYOUT = 
        AprilTagFields.k2025ReefscapeWelded.loadAprilTagLayoutField();
}