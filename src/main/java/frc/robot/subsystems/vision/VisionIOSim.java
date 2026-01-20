package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class VisionIOSim implements VisionIO {
    private CommandSwerveDrivetrain drivetrain;

    public VisionIOSim(CommandSwerveDrivetrain drivetrain) {
        // Optionally, you can use the drivetrain reference to simulate tx based on
        this.drivetrain = drivetrain;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        List<Integer> visibleTagIds = new ArrayList<>();
        List<Double> visibleTagDistances = new ArrayList<>();
        List<Double> visibleTagTxs = new ArrayList<>();
        Pose2d robotPose = drivetrain.getState().Pose;

        for (AprilTag tag : AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark).getTags()) {
            Pose2d tagPose = tag.pose.toPose2d();
            double distance = robotPose.getTranslation().getDistance(tagPose.getTranslation());
            double angleToTag = tagPose.getTranslation().minus(robotPose.getTranslation()).getAngle().getDegrees()
                    - robotPose.getRotation().getDegrees();

            // Simple visibility check (e.g., within 5 meters and +/- 30 degrees)
            if (distance < 5.0 && Math.abs(angleToTag) < 30.0) {
                visibleTagIds.add(tag.ID);
                visibleTagDistances.add(distance);
                visibleTagTxs.add(angleToTag);
            }
        }

        // Populate inputs arrays
        inputs.tagIds = visibleTagIds.stream().mapToInt(i -> i).toArray();
        inputs.tagDistances = visibleTagDistances.stream().mapToDouble(d -> d).toArray();
        inputs.tagTxncs = visibleTagTxs.stream().mapToDouble(d -> d).toArray();

        if (inputs.tagIds.length > 0) {
            inputs.tx = inputs.tagTxncs[0];
            inputs.hasTag = true;
        } else {
            inputs.tx = 0.0;
            inputs.hasTag = false;
        }
    }
}