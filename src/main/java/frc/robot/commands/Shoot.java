// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constents;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Shoot extends Command {
  Shooter m_shooter;
  Vision m_vision;
  double m_flywheelSpeed = .75;
  double m_hoodWheelSpeed = 1;

  /** Creates a new Shoot. */
  public Shoot(Shooter shooter, Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("FlyWheel Speed", m_flywheelSpeed);
    SmartDashboard.putNumber("Hood Wheel Speed", m_hoodWheelSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distFromTag = m_vision.getDist();
    if (distFromTag <= Constents.targetDistanceToTag - Constents.offsetInMeters || distFromTag >= Constents.targetDistanceToTag + Constents.offsetInMeters){
    m_flywheelSpeed = SmartDashboard.getNumber("FlyWheel Speed", m_flywheelSpeed);
    m_hoodWheelSpeed = SmartDashboard.getNumber("Hood Wheel Speed", m_hoodWheelSpeed);
    m_shooter.Shoot(m_flywheelSpeed, m_hoodWheelSpeed);
  }
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
