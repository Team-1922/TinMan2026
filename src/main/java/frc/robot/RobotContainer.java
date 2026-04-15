// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoAlign;
import frc.robot.commands.RetractCollector;
import frc.robot.commands.ReverseCollector;
import frc.robot.commands.SpinCollectorBars;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Localization;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Signaling;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Collector;
import frc.robot.commands.Collect;
import frc.robot.commands.HalfCollect;
import frc.robot.commands.BandShoot;

import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
  private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                      // speed
  private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond); // 1 rotation per second
                                                                                              // max angular velocity
  private double collectingSpeedScalar = 1;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.12).withRotationalDeadband(MaxAngularRate * 0.12) // Add a 12% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final Telemetry logger = new Telemetry(MaxSpeed);
  public final CommandXboxController DriverController = new CommandXboxController(0);
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public final Shooter shooter = new Shooter();
  public final Spindexer spindexer = new Spindexer();
  public final Feeder feeder = new Feeder();
  public final Localization localization = new Localization(drivetrain);
  public final Collector collector = new Collector();
  public final Signaling signaling = new Signaling(DriverController);
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    configureBindings();

    NamedCommands.registerCommand(
      "alignAndShoot",
      new ParallelCommandGroup(
        new AutoAlign(drivetrain, localization, signaling, true),
        new BandShoot(
          shooter,
          feeder,
          spindexer,
          localization,
          BandShoot.ShootActions.Shoot)));
    NamedCommands.registerCommand(
      "shoot",
      new BandShoot(
        shooter,
        feeder,
        spindexer,
        localization,
        BandShoot.ShootActions.JustShoot));
    NamedCommands.registerCommand(
      "autoAlign",
      new AutoAlign(drivetrain, localization, signaling, false));
    NamedCommands.registerCommand("collect", new Collect(collector));
    NamedCommands.registerCommand("zero", drivetrain.runOnce(drivetrain::seedFieldCentric));
    NamedCommands.registerCommand("Half Collector", new HalfCollect(collector));
    NamedCommands.registerCommand("Spin Collector", new SpinCollectorBars(collector));
    NamedCommands.registerCommand("Full Collect", new RetractCollector(collector));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    LimelightHelpers.setCameraPose_RobotSpace(
      Constants.frontLimelightName,
      Units.inchesToMeters(3.5),
      Units.inchesToMeters(-7.5),
      Units.inchesToMeters(20.25),
      0,
      30,
      0);

    LimelightHelpers.setCameraPose_RobotSpace(
      Constants.rightLimelightName,
      -0.168275,
      -0.034925,
      0.669925,
      0,
      0,
      -90);
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
      // Drivetrain will execute this command periodically

      Commands.run( () -> drivetrain.Move(
                      (-DriverController.getLeftY() * MaxSpeed * Constants.kdriveSpeedScaler * collectingSpeedScalar), // Drive forward with negative Y (forward)
                      (-DriverController.getLeftX() * MaxSpeed * Constants.kdriveSpeedScaler * collectingSpeedScalar), // Drive left with negative X (left)
                      (-DriverController.getRightX() * MaxAngularRate * Constants.kdriveSpeedScaler) // Drive counterclockwise with negative
                                                                            // X (left)
                      )
      ));

    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
      drivetrain.applyRequest(() -> idle).ignoringDisable(true));

    DriverController.povRight().whileTrue(drivetrain.applyRequest(() -> brake));

    DriverController.leftTrigger().whileTrue(new Collect(collector));

    DriverController.rightTrigger().whileTrue(
      new ParallelCommandGroup(
        new AutoAlign(drivetrain, localization, signaling, true),
        new BandShoot(shooter, feeder, spindexer, localization, BandShoot.ShootActions.Shoot)));

    DriverController.povDown().whileTrue(
      new RetractCollector(collector));

    DriverController.povLeft().whileTrue(
      new HalfCollect(collector));

    DriverController.rightBumper().whileTrue(
      new BandShoot(shooter, feeder, spindexer, localization, BandShoot.ShootActions.Shuttle));

    DriverController.x().whileTrue(
      new ReverseCollector(collector));

    DriverController.b().whileTrue(
      new BandShoot(shooter, feeder, spindexer, localization, BandShoot.ShootActions.JustShoot));

    DriverController.leftBumper().whileTrue(
       Commands.run( () -> collectingSpeedScalar = .2)
    );

    DriverController.leftBumper().whileFalse(
      Commands.run( () -> collectingSpeedScalar = 1)
    );

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    DriverController.back().and(DriverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    DriverController.back().and(DriverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    DriverController.start().and(DriverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    DriverController.start().and(DriverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // Reset the field-centric heading on left bumper press.
    DriverController.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
