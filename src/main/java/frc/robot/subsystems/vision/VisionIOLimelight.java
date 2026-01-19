package frc.robot.subsystems.vision;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.RawFiducial;

public class VisionIOLimelight implements VisionIO {
    private final String limelightName;

    public VisionIOLimelight(String limelightName) {
        this.limelightName = limelightName;
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(limelightName);
        if (fiducials.length > 0) {
            inputs.tagIds = new int[fiducials.length];
            inputs.tagDistances = new double[fiducials.length];
            inputs.tagTxs = new double[fiducials.length];
            for (int i = 0; i < fiducials.length; i++) {
                inputs.tagIds[i] = fiducials[i].id;
                inputs.tagDistances[i] = fiducials[i].distToCamera;
                inputs.tagTxs[i] = fiducials[i].txnc;
            }
            inputs.hasTag = true;
        } else {
            inputs = new VisionIOInputs(); // Reset inputs if no fiducial is detected
        }
    }
}