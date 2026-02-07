// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {
 private final Shooter m_shooter;
 private final Vision m_vision;
 private double m_rps;

  /** Creates a new Shoot. */
  public Shoot(Shooter shooter, Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Shooter RPM", m_rps);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distFromTag = m_vision.getDist();

    if (distFromTag <= Constants.targetDistanceToTag - Constants.offsetInMeters || distFromTag >= Constants.targetDistanceToTag + Constants.offsetInMeters){
      m_rps = SmartDashboard.getNumber("Shooter RPM", m_rps);
      
      m_shooter.run(m_rps);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
