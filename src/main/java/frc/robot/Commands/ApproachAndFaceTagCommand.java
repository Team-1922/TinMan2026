package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;
import java.util.function.DoubleSupplier;

public class ApproachAndFaceTagCommand extends Command {
    private final MoveSwerveCommand moveCommand;
    private final Vision vision;
    private final double targetDistanceMeters;
    private final int targetTagId;
    private final double maxSpeed;
    private final double maxAngularRate;

    public ApproachAndFaceTagCommand(CommandSwerveDrivetrain drivetrain,
            Vision vision, int targetTagId,
            double targetDistanceInMeters,
            double maxSpeed,
            double maxAngularRate) {
        this.vision = vision;
        this.targetDistanceMeters = targetDistanceInMeters;
        this.targetTagId = targetTagId;
        this.maxSpeed = maxSpeed;
        this.maxAngularRate = maxAngularRate;

        DoubleSupplier forwardSpeed = () -> {
            double distanceToTag = vision.getDistanceToTag(targetTagId);
            if (distanceToTag == 0.0){
                return 0.0;
            }
            double error = distanceToTag - targetDistanceInMeters;
            return Math.max(Math.min(error * 1, 1.0), -1.0) * maxSpeed;
        };

        DoubleSupplier rotationalRate = () -> {
            double horizontalOffsetToTag = vision.getTxncForTag(targetTagId);
            if (horizontalOffsetToTag == 0.0){
                return maxAngularRate;
            }                
            return -horizontalOffsetToTag * 0.02 *maxAngularRate;
        };

        this.moveCommand = new MoveSwerveCommand(
                drivetrain,
                forwardSpeed,
                () -> 0.0,
                rotationalRate,
                false // Robot-centric
        );
        addRequirements(drivetrain, vision);
    }

    @Override
    public void initialize() {
        moveCommand.initialize();
    }

    @Override
    public void execute() {
        LimelightHelpers.SetFiducialIDFiltersOverride(getName(), new int[] {targetTagId});
        moveCommand.execute();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(vision.getDistanceToTag(targetTagId) - targetDistanceMeters) < 0.05
                && Math.abs(vision.getTxncForTag(targetTagId)) < 2.0;
    }

    @Override
    public void end(boolean interrupted) {
        moveCommand.end(interrupted);
    }
}