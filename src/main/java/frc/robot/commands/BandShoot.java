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
import frc.robot.subsystems.Localization;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BandShoot extends Command {
  private double m_spindexerRps = 45;
  private double m_feederRps = 60;
  private double m_shooterRps = 20;
  private double m_shuttleRps = 30;
  private double m_yawThreshold = .06;
  private boolean m_isReadyToShoot;
  private boolean m_requireAlign = true;
  private boolean m_isTuning = true;
  private final Shooter m_shooter;
  private final Spindexer m_spindexer;
  private final Feeder m_feeder;
  private final Localization m_localization;
  private final double m_shooterVelocityThreshold = 2;
  private final double m_feederVelocityThreshold = 1;
  private final double KPForRPS = 4.75; 
  private final double m_minShooterRps = 10.025; //rps at min distance

  private ShootActions m_shootAction = ShootActions.Shoot;
  public enum ShootActions {
    Shoot,
    Shuttle,
    JustShoot
  }

  /** Creates a new BandShoot. */
  public BandShoot(
      Shooter shooter,
      Feeder feeder,
      Spindexer spindexer,
      Localization localization,
      ShootActions shootAction
      ) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_feeder = feeder;
    m_spindexer = spindexer;
    m_localization = localization;
    m_shootAction = shootAction;
    SmartDashboard.putNumber("Shooter RPS", m_shooterRps);
    SmartDashboard.putNumber("Spindexer RPS", m_spindexerRps);
    SmartDashboard.putNumber("Feeder RPS", m_feederRps);
    SmartDashboard.putBoolean("Requires Align", m_requireAlign);
    SmartDashboard.putNumber("Yaw Threshold", m_yawThreshold);
    addRequirements(m_shooter, m_feeder, m_spindexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_isReadyToShoot = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distFromHub = m_localization.distFromHub();
    m_shooterRps = m_minShooterRps + KPForRPS * (distFromHub);
    SmartDashboard.putNumber("Distance From Hub", distFromHub);
    m_isTuning = SmartDashboard.getBoolean("isTuning", m_isTuning);
    if(m_isTuning){
      m_shooterRps = SmartDashboard.getNumber("Shooter RPS", m_shooterRps);
      m_spindexerRps = SmartDashboard.getNumber("Spindexer RPS", m_spindexerRps);
      m_feederRps = SmartDashboard.getNumber("Feeder RPS", m_feederRps);
      m_requireAlign = SmartDashboard.getBoolean("Requires Align", m_requireAlign);
      m_yawThreshold = SmartDashboard.getNumber("Yaw Threshold", m_yawThreshold);
    }

    if(m_shootAction == ShootActions.Shoot) {
      m_requireAlign = true;
    }
    if(m_shootAction == ShootActions.JustShoot){
      m_requireAlign = false;

    }

    m_shooter.setTargetRps(m_shooterRps);

    if (
        !m_requireAlign
        || (
          distFromHub < Constants.maxTargetDistanceToHub
          && Math.abs(m_localization.getM_errorYaw()) < m_yawThreshold
        )
    ) {
      if (m_shooter.getVelocity() >= m_shooterRps - m_shooterVelocityThreshold) {
        m_isReadyToShoot = true;
      }

      if (m_isReadyToShoot) {     

        m_feeder.setTargetRps(m_feederRps);

        if (m_feeder.getVelocity() >= m_feederRps - m_feederVelocityThreshold) {
          m_spindexer.setTargetRps(m_spindexerRps);
        }        
      }
    } else if (m_feeder.getVelocity() > 0) {
      m_feeder.stop();
      m_spindexer.setTargetRps(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    m_spindexer.setTargetRps(0);
    m_feeder.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
