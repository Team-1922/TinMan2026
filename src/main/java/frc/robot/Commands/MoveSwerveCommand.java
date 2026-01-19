package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import java.util.function.DoubleSupplier;

public class MoveSwerveCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final DoubleSupplier xVelocity;
    private final DoubleSupplier yVelocity;
    private final DoubleSupplier rotationalRate;
    private final boolean fieldCentric;

    public MoveSwerveCommand(
        CommandSwerveDrivetrain drivetrain,
        DoubleSupplier xVelocity,
        DoubleSupplier yVelocity,
        DoubleSupplier rotationalRate,
        boolean fieldCentric
    ) {
        this.drivetrain = drivetrain;
        this.xVelocity = xVelocity;
        this.yVelocity = yVelocity;
        this.rotationalRate = rotationalRate;
        this.fieldCentric = fieldCentric;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        SwerveRequest swerveRequest = fieldCentric
            ? new SwerveRequest.FieldCentric().withVelocityX(xVelocity.getAsDouble())
                .withVelocityY(yVelocity.getAsDouble())
                .withRotationalRate(rotationalRate.getAsDouble())
            : new SwerveRequest.RobotCentric().withVelocityX(xVelocity.getAsDouble())
                .withVelocityY(yVelocity.getAsDouble())
                .withRotationalRate(rotationalRate.getAsDouble());

        drivetrain.applyRequest(() -> swerveRequest);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.idle();
    }
}