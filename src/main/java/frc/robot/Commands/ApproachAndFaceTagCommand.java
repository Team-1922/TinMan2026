package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;

import java.util.OptionalDouble;
import java.util.function.DoubleSupplier;

public class ApproachAndFaceTagCommand extends Command {
    private static final double X_VELOCITY_KP = 1.0;
    private static final double Y_VELOCITY_KP = 1.0;
    private static final double ROTATION_KP = 0.02;
    private static final double MIN_NORMALIZED_SPEED = -1.0;
    private static final double MAX_NORMALIZED_SPEED = 1.0;
    private static final double DISTANCE_TOLERANCE_METERS = 0.05;
    private static final double HORIZONTAL_TOLERANCE_DEGREES = 5.0;
    private static final double SEARCH_SPIN_RATE = 1.0; // Fraction of maxAngularRate

    private final MoveSwerveCommand moveCommand;
    private final Vision vision;
    private final double targetDistanceToTagInMeters;
    private final int targetTagId;

    public ApproachAndFaceTagCommand(CommandSwerveDrivetrain drivetrain,
            Vision vision, int targetTagId,
            double targetDistanceToTagInMeters,
            double maxSpeed,
            double maxAngularRate) {
        this.vision = vision;
        this.targetDistanceToTagInMeters = targetDistanceToTagInMeters;
        this.targetTagId = targetTagId;

        DoubleSupplier xVelocity = () -> {
            double distanceToCoverInMeters = getDistanceToCover();
            return MathUtil.clamp(distanceToCoverInMeters * X_VELOCITY_KP, MIN_NORMALIZED_SPEED, MAX_NORMALIZED_SPEED) * maxSpeed;
        };

        DoubleSupplier rotationalRate = () -> {
            OptionalDouble horizontalOffsetToTag = vision.getTxncForTag(targetTagId);
            if (horizontalOffsetToTag.isEmpty()) {
                return maxAngularRate * SEARCH_SPIN_RATE;
            }
            return -horizontalOffsetToTag.getAsDouble() * ROTATION_KP * maxAngularRate;
        };

        this.moveCommand = new MoveSwerveCommand(
                drivetrain,
                xVelocity,
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
        LimelightHelpers.SetFiducialIDFiltersOverride(getName(), new int[] { targetTagId });
        moveCommand.execute();
    }

    @Override
    public boolean isFinished() {
        OptionalDouble horizontalOffsetToTag = vision.getTxncForTag(targetTagId);

        return Math.abs(getDistanceToCover()) < DISTANCE_TOLERANCE_METERS
                && horizontalOffsetToTag.isPresent()
                && Math.abs(horizontalOffsetToTag.getAsDouble()) < HORIZONTAL_TOLERANCE_DEGREES;
    }

    @Override
    public void end(boolean interrupted) {
        moveCommand.end(interrupted);
    }

    private double getDistanceToCover() {
        OptionalDouble currentDistanceToTagInMeters = vision.getDistanceToTag(targetTagId);
        return currentDistanceToTagInMeters.orElse(0.0) - targetDistanceToTagInMeters;
    }
}