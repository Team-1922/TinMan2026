// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Localization;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {
  CommandSwerveDrivetrain m_drivetrain;
  Localization m_localization;
   
  double m_xKp = 5;
  double m_yKp = 5;
  double m_yawKp = 1;
  double m_alianceSign = 1;

  /** Creates a new AutoAlign. */
  public AutoAlign(CommandSwerveDrivetrain drivetrain, Localization localization) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    m_drivetrain = drivetrain;
    m_localization = localization;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_alianceSign = DriverStation.getAlliance().get() == Alliance.Blue ? 1 : -1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double vX = m_localization.getM_errorX() * m_xKp * m_alianceSign;
    double vY = m_localization.getM_errorY() * m_yKp * m_alianceSign;
    double vYaw = m_localization.getM_errorYaw() * m_yawKp;
    
    m_drivetrain.Move(vX, vY, vYaw);
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
