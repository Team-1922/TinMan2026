// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.lang.reflect.Array;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {
  Vision m_vision;
  CommandSwerveDrivetrain m_Drivetrain;
   double m_offsetInMeters = Meters.of(.5).in(Meters);
    double m_targetDistanceToTag = Meters.of(2.7).in(Meters);
    double m_percentXOffsetToTag = 1.5;
    int m_blueHubMiddleTag = 10;
    int  m_redHubMiddleTag = 26;
    double m_proportionalXSpeed = 1.5;
    double m_proportionalRotatinalRate = .15;


  /** Creates a new AutoAlign. */
  public AutoAlign(Vision vision, CommandSwerveDrivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_vision = vision;
    m_Drivetrain = drivetrain; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight-front", new int[]{m_redHubMiddleTag, m_blueHubMiddleTag});

    double speed = calculateSpeed();
    double rotationRate = calculateRotatinalRate();

    m_Drivetrain.Move(speed, 0, rotationRate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  LimelightHelpers.SetFiducialIDFiltersOverride("limelight-front", new int[]{});
  m_Drivetrain.Move(0, 0, 0);
}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double calculateSpeed() {
    double speed = 0;

    double distFromTag = m_vision.getDist();
      if (distFromTag <= m_targetDistanceToTag - m_offsetInMeters || distFromTag >= m_targetDistanceToTag + m_offsetInMeters) {
        speed = (distFromTag - m_targetDistanceToTag) * m_proportionalXSpeed; 
    }

    return speed;
  }

  private double calculateRotatinalRate() {
    double rotationalRate = 0;
    
    if (m_vision.getTx() <= -m_percentXOffsetToTag || m_vision.getTx() >= m_percentXOffsetToTag) {
      rotationalRate = -m_vision.getTx() * m_proportionalRotatinalRate;
    }

    return rotationalRate;
  }
}
