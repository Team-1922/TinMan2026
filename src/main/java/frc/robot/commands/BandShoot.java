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
  private boolean m_requireAlign = true;
  private double m_shooterRps = 20;
  private final Shooter m_shooter;
  private final Spindexer m_spindexer;
  private final Feeder m_feeder;
  private final Localization m_localization;
  private final double m_shooterVelocityThreshold = 2;
  private final double m_feederVelocityThreshold = 1;
  private final double m_kpForRps = 4.25; 
  private final double m_minShooterRps = 10.0; //rps at 0 meters from the center of the hub
  private final double m_spindexerRps = 45;
  private final double m_feederRps = 60;
  private final double m_shuttleRps = 30;

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
    /* 
    SmartDashboard.putNumber("Shooter RPS", m_shooterRps);
    SmartDashboard.putNumber("Feeder RPS", m_feederRps);
    SmartDashboard.putNumber("Spindexer RPS", m_spindexerRps);
    SmartDashboard.putBoolean("Requires Align", m_requireAlign);
    SmartDashboard.putNumber("Yaw Threshold", Constants.kyawThreshold);
    */
    addRequirements(m_shooter, m_feeder, m_spindexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_localization.setTarget();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //SmartDashboard.putNumber("Distance From target", distFromTarget);
    if (!m_localization.hasTarget()) {
      return;
    }

    double distFromTarget = m_localization.distFromTarget();
    m_shooterRps = m_minShooterRps + m_kpForRps * (distFromTarget);

    if(m_shootAction == ShootActions.Shoot) {
      m_requireAlign = true;
    }
    else if(m_shootAction == ShootActions.JustShoot){
      m_requireAlign = false;
    }
    else if(m_shootAction == ShootActions.Shuttle){
      m_shooterRps = m_shuttleRps;
      m_requireAlign = false;
    }

    m_shooter.setTargetRps(m_shooterRps);

    if (
      !m_requireAlign
      || ( 
        distFromTarget < Constants.maxTargetDistanceToTarget
        && Math.abs(m_localization.getM_errorYaw()) < Constants.kshooterYawThreshold
      )
    ) {

      if (m_shooter.getVelocity() >= m_shooterRps - m_shooterVelocityThreshold) {     

        m_feeder.setTargetRps(m_feederRps);
        m_spindexer.setTargetRps(m_spindexerRps);

      }
    } else if (m_feeder.getVelocity() > 0) {
      m_feeder.stop();
      m_spindexer.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stop();
    m_spindexer.stop();
    m_feeder.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
