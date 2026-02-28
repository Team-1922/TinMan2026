// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Collector;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Collect extends Command {
  /** Creates a new Collect. */
  Collector m_collector;
  private double m_collectorRps = 40;
  private double m_collectorAngle = 180;

  public Collect(Collector collector) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_collector = collector;
    addRequirements(collector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      m_collector.setTargetRps(m_collectorRps);
      SmartDashboard.putNumber("Collector RPS", m_collectorRps);
      m_collector.deploy(m_collectorAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_collectorRps = SmartDashboard.getNumber("Collector RPS", m_collectorRps);
    m_collector.collect();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_collector.stopCollector();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
