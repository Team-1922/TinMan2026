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
  double m_alianceSign = 1;


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
    m_alianceSign = DriverStation.getAlliance().get() == Alliance.Blue ? 1 : -1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double vX = 0;
    double vY = 0;
    if((m_localization.distFromHub() > Constants.maxTargetDistanceToHub || m_normalAutoAlign)
        && m_localization.getM_errorYaw() > Constants.kyawThreshold
    ){
      vX = m_localization.getM_errorX() * m_xKp * m_alianceSign;
      vY = m_localization.getM_errorY() * m_yKp * m_alianceSign;
      double vYaw = m_localization.getM_errorYaw() * m_yawKp;
      m_drivetrain.Move(vX, vY, vYaw);
    } else {
      m_drivetrain.applyRequest( new SwerveDriveBrake());
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
