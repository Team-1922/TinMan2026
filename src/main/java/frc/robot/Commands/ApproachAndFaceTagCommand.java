package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;
import java.util.function.DoubleSupplier;

public class ApproachAndFaceTagCommand extends Command {
    private final MoveSwerveCommand moveCommand;
    private final Vision vision;
    private final double targetDistanceMeters;
    private final int targetTagId;

    public ApproachAndFaceTagCommand(CommandSwerveDrivetrain drivetrain, Vision vision, int targetTagId,
            double targetDistanceMeters) {
        this.vision = vision;
        this.targetDistanceMeters = targetDistanceMeters;
        this.targetTagId = targetTagId;

        DoubleSupplier forwardSpeed = () -> {
            if (vision.getDistanceToTag(targetTagId) == 0.0)
                return 0.0;
            double error = vision.getDistanceToTag(targetTagId) - targetDistanceMeters;
            return Math.max(Math.min(error * 0.5, 1.0), -1.0);
        };

        DoubleSupplier rotationalRate = () -> {
            if (vision.getTxToTag(targetTagId)== 0.0)
                return 0.2; // Spin slowly
            return -vision.getTxToTag(targetTagId) * 0.02;
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
        moveCommand.execute();
    }

    @Override
    public boolean isFinished() {
        return Math.abs(vision.getDistanceToTag(targetTagId) - targetDistanceMeters) < 0.05
                && Math.abs(vision.getTxToTag(targetTagId)) < 2.0;
    }

    @Override
    public void end(boolean interrupted) {
        moveCommand.end(interrupted);
    }
}