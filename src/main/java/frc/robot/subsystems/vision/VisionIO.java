package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    @AutoLog
    public class VisionIOInputs {
        public int[] tagIds = new int[0];
        public double[] tagDistances = new double[0];
        public double[] tagTxs = new double[0];
        public boolean hasTag = false;
    }

    void updateInputs(VisionIOInputs inputs);
}