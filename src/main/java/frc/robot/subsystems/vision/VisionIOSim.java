package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;

import java.util.Random;
import java.util.function.Supplier;

public class VisionIOSim implements VisionIO {
    
    private final Supplier<Pose2d> poseSupplier;
    private final Random random = new Random();
    
    // Simulated measurement noise
    private static final double XY_NOISE_METERS = 0.02; // 2cm noise
    private static final double THETA_NOISE_DEGREES = 0.5; // 0.5 degree noise
    
    public VisionIOSim(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        Pose2d currentPose = poseSupplier.get();
        
        // Add simulated noise to pose
        Pose2d noisyPose = new Pose2d(
            currentPose.getX() + (random.nextGaussian() * XY_NOISE_METERS),
            currentPose.getY() + (random.nextGaussian() * XY_NOISE_METERS),
            currentPose.getRotation().plus(Rotation2d.fromDegrees(random.nextGaussian() * THETA_NOISE_DEGREES))
        );
        
        // Find visible AprilTags using the noisy pose
        var visibleTag = VisionConstants.FIELD_LAYOUT.getTags().stream()
            .filter(tag -> {
                Pose2d tagPose = tag.pose.toPose2d();
                double distance = noisyPose.getTranslation().getDistance(tagPose.getTranslation());
                
                // Check if tag is in range and roughly in front of robot (within ±90 degrees)
                if (distance >= VisionConstants.DETECTION_RANGE_METERS) {
                    return false;
                }
                
                double angleToTag = Math.atan2(
                    tagPose.getY() - noisyPose.getY(),
                    tagPose.getX() - noisyPose.getX()
                );
                double robotAngle = noisyPose.getRotation().getRadians();
                double relativeAngle = Math.abs(angleToTag - robotAngle);
                
                // Normalize to [-pi, pi]
                while (relativeAngle > Math.PI) relativeAngle -= 2 * Math.PI;
                while (relativeAngle < -Math.PI) relativeAngle += 2 * Math.PI;
                
                return Math.abs(relativeAngle) < Math.toRadians(90); // Within FOV
            })
            .min((t1, t2) -> {
                double d1 = noisyPose.getTranslation().getDistance(t1.pose.toPose2d().getTranslation());
                double d2 = noisyPose.getTranslation().getDistance(t2.pose.toPose2d().getTranslation());
                return Double.compare(d1, d2);
            });
        
        if (visibleTag.isPresent()) {
            var tag = visibleTag.get();
            Pose3d tagPose3d = tag.pose;
            Pose2d tagPose2d = tagPose3d.toPose2d();
            
            // Calculate relative position
            double deltaX = tagPose2d.getX() - noisyPose.getX();
            double deltaY = tagPose2d.getY() - noisyPose.getY();
            double distance = Math.hypot(deltaX, deltaY);
            
            // Calculate angles relative to robot heading
            double angleToTarget = Math.atan2(deltaY, deltaX);
            double robotAngle = noisyPose.getRotation().getRadians();
            double relativeAngle = angleToTarget - robotAngle;
            
            double tx = Math.toDegrees(relativeAngle);
            double ty = Math.toDegrees(Math.atan2(
                tagPose3d.getZ() - VisionConstants.CAMERA_HEIGHT_METERS, 
                distance)) + VisionConstants.CAMERA_PITCH_DEGREES;
            double ta = Math.max(0.1, 8.0 / (distance * distance)); // Simulated area
            
            // Update inputs with noisy pose
            inputs.hasTargets = true;
            inputs.tx = tx;
            inputs.ty = ty;
            inputs.ta = ta;
            inputs.targetId = tag.ID;
            inputs.botpose = noisyPose; // Use noisy measurement
            inputs.timestamp = Timer.getFPGATimestamp();
            inputs.latency = 35.0 + (random.nextDouble() * 10.0); // 35-45ms latency
            inputs.numTargets = 1;
            
        } else {
            // No targets visible
            inputs.hasTargets = false;
            inputs.timestamp = Timer.getFPGATimestamp();
        }
    }
}
