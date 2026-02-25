// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.Shoot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Localization;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
//import frc.robot.subsystems.Collector;
//import frc.robot.commands.Collect;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.IdleSpindexer;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.12).withRotationalDeadband(MaxAngularRate * 0.12) // Add a 12% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController DriverController = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Shooter shooter = new Shooter();
    public final Spindexer spindexer = new Spindexer();
    public final Feeder feeder = new Feeder();
    public final Localization localization = new Localization(drivetrain);
    //public final Collector collector = new Collector();  
    private final SendableChooser<Command> autoChooser;

//    public final Collect collect = new Collect(collector);
    public final IdleSpindexer idleSpindexer = new IdleSpindexer(spindexer);
    public final AutoAlign autoAlign = new AutoAlign(drivetrain, localization);
    public final AutoAlign autoAutoAlign = new AutoAlign(drivetrain, localization);
    public final Shoot shoot = new Shoot(shooter, feeder, spindexer, localization);
    public final Shoot autoShoot = new Shoot(shooter, feeder, spindexer, localization);

    public RobotContainer() {
        configureBindings();
        
        NamedCommands.registerCommand("shoot", new ParallelCommandGroup(autoAutoAlign, autoShoot));
        NamedCommands.registerCommand("AutoAlign", autoAutoAlign);
        NamedCommands.registerCommand("setBotPose", drivetrain.runOnce(drivetrain::seedFieldCentric));
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }
    
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically

                drivetrain.applyRequest(() -> drive
                        .withVelocityX(-DriverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                        .withVelocityY(-DriverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-DriverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        DriverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        DriverController.b().whileTrue(drivetrain.applyRequest(() -> point
                .withModuleDirection(new Rotation2d(-DriverController.getLeftY(), -DriverController.getLeftX()))));
    //    DriverController.leftTrigger().whileTrue(collect);
        DriverController.rightTrigger().whileTrue( 
            new ParallelCommandGroup(
                autoAlign, 
                new Shoot(shooter, feeder, spindexer, localization)
        ));
        DriverController.rightBumper().whileTrue(
            new Shoot(shooter, feeder, spindexer, localization)
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        DriverController.back().and(DriverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        DriverController.back().and(DriverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        DriverController.start().and(DriverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        DriverController.start().and(DriverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        DriverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
