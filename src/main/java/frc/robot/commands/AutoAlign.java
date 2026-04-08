// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Localization;
import frc.robot.subsystems.Signaling;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {
  CommandSwerveDrivetrain m_drivetrain;
  Localization m_localization;
  Signaling m_signaling;
  Boolean m_normalAutoAlign;
  double m_xKp = 5;
  double m_yKp = 5;
  double m_yawKp = 3.8;
  double m_allianceSign = 1;
  private final SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake();


  /** Creates a new AutoAlign. */
  public AutoAlign(CommandSwerveDrivetrain drivetrain, Localization localization, Signaling signaling, Boolean normalAutoAlign) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
    m_localization = localization;
    m_signaling = signaling;
    m_normalAutoAlign = normalAutoAlign;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_allianceSign = DriverStation.getAlliance().get() == Alliance.Blue ? 1 : -1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double vX = 0;
    double vY = 0;
    double vYaw = 0;
    boolean isAligned = true;

    if(
        m_normalAutoAlign
        || m_localization.distFromHub() > Constants.maxTargetDistanceToHub
    ) {
      vX = m_localization.getM_errorX() * m_xKp * m_allianceSign;
      vY = m_localization.getM_errorY() * m_yKp * m_allianceSign;
      isAligned = false;
    } 

    if(Math.abs(m_localization.getM_errorYaw()) > Constants.kyawThreshold) {
      vYaw = m_localization.getM_errorYaw() * m_yawKp;
      isAligned = false;
    }

    if(isAligned) {
      m_drivetrain.setControl(m_brake);
    } else {
      m_drivetrain.Move(vX, vY, vYaw);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
