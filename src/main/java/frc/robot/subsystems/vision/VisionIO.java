package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean hasTargets = false;
        public double tx = 0.0;  // Horizontal offset to target (degrees)
        public double ty = 0.0;  // Vertical offset to target (degrees)
        public double ta = 0.0;  // Target area (0-100%)
        public int targetId = -1; // AprilTag ID
        public Pose2d botpose = null; // Robot pose from vision
        public double timestamp = 0.0; // Timestamp of measurement
        public double latency = 0.0; // Pipeline latency (ms)
        public int numTargets = 0; // Number of visible targets
        public double[] botpose3d = new double[0]; // Full 3D pose [x,y,z,roll,pitch,yaw]
    }

    /**
     * Updates the set of loggable inputs.
     */
    public default void updateInputs(VisionIOInputs inputs) {}
}
