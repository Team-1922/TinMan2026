// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Localization;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {
 private final Shooter m_shooter;
 private final Vision m_vision;
 private double m_spindexerRps = 3;
 private double m_feederRps = 50;
 private double m_shooterRps = 23.8;
 private final Spindexer m_spindexer;
 private final Feeder m_feeder;
 private final Localization m_localization;
 private final double m_shooterSpeedThreshold = 5;

  /** Creates a new Shoot. */
  public Shoot(Shooter shooter, Vision vision, Feeder feeder, Spindexer spindexer, Localization localization) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_vision = vision;
    m_feeder = feeder;
    m_spindexer = spindexer;
    m_localization = localization;
    addRequirements(m_shooter, m_feeder, m_spindexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Shooter RPS", m_shooterRps);
    SmartDashboard.putNumber("Spindexer RPS", m_spindexerRps);
    SmartDashboard.putNumber("Feeder RPS", m_feederRps);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distFromHub = m_localization.distFromHub();
    m_shooter.setTargetRps(m_shooterRps);
    m_spindexer.setTargetRps(m_spindexerRps);
    SmartDashboard.putNumber("Shooter Velocity", m_shooter.getVelocity());
    if (Math.abs(distFromHub - Constants.targetDistanceToHub) < Constants.offsetInMeters){
      if(m_shooter.getVelocity() >= m_shooterRps - m_shooterSpeedThreshold){
        m_feeder.setTargetRps(m_feederRps);
      } else{
        m_feeder.stop();
      }
     } else if(m_feeder.getSpeed() > 0){
      m_feeder.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    m_spindexer.setIdleSpeed();
    m_feeder.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
