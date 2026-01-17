package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Vision extends SubsystemBase {
    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    private final CommandSwerveDrivetrain drivetrain;
    private final AprilTagFieldLayout fieldLayout;
    
    public Vision(VisionIO io, CommandSwerveDrivetrain drivetrain) {
        this.io = io;
        this.drivetrain = drivetrain;
        this.fieldLayout = AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026ChargedUp);
    }
    
    @Override
    public void periodic() {
        // Update inputs from IO layer
        io.updateInputs(inputs);

        fieldLayout.getTagPose(1);

        Logger.processInputs("Vision", inputs);
        
        // Business logic: Update drivetrain with vision measurements
        if (inputs.hasTargets && inputs.botpose != null) {
            var stdDevs = VecBuilder.fill(
                VisionConstants.DEFAULT_XY_STD_DEV, 
                VisionConstants.DEFAULT_XY_STD_DEV, 
                Math.toRadians(VisionConstants.DEFAULT_THETA_STD_DEV_DEGREES)
            );
            
            drivetrain.addVisionMeasurement(
                inputs.botpose,
                inputs.timestamp - (inputs.latency / 1000.0),
                stdDevs
            );
        }
    }
}