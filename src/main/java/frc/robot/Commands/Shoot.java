// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {
  public final Shooter m_shooter;
  double m_tSpeed = .75;
  double m_bSpeed = 1;

  /** Creates a new Shoot. */
  public Shoot(Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("FlyWheel Speed", m_tSpeed);
    SmartDashboard.putNumber("Hood Wheel Speed", m_bSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_tSpeed = SmartDashboard.getNumber("FlyWheel Speed", m_tSpeed);
    m_bSpeed = SmartDashboard.getNumber("Hood Wheel Speed", m_bSpeed);
    m_shooter.Shoot(m_tSpeed, m_bSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.Shoot(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
