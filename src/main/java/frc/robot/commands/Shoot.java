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

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {
 private final Shooter m_shooter;
 private final Vision m_vision;
 private double m_spindexerRps = 1;
 private double m_feederRps = 18;
 private double m_shooterRps = 20;
 private final Spindexer m_spindexer;
 private final Feeder m_feeder;
 private final double m_shooterSpeedOffset = .1;

  /** Creates a new Shoot. */
  public Shoot(Shooter shooter, Vision vision, Feeder feeder, Spindexer spindexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_vision = vision;
    m_feeder = feeder;
    m_spindexer = spindexer;
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
    double distFromTag = m_vision.getDist();

   // if (distFromTag <= Constants.targetDistanceToTag - Constants.offsetInMeters || distFromTag >= Constants.targetDistanceToTag + Constants.offsetInMeters){
        // m_shooterRps = SmartDashboard.getNumber("Shooter RPS", m_shooterRps);
        // m_spindexerRps = SmartDashboard.getNumber("Spindexer RPS", m_spindexerRps);
        // m_feederRps = SmartDashboard.getNumber("Feeder RPS", m_feederRps);
    
    m_shooter.setTargetRps(m_shooterRps);
    m_spindexer.setTargetRps(m_spindexerRps);
    SmartDashboard.putNumber("Shooter Velocity", m_shooter.getVelocity());
    if(m_shooter.getVelocity() >= m_shooterRps - m_shooterSpeedOffset){
      m_feeder.setTargetRps(m_feederRps);
    }
      
   // }
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
