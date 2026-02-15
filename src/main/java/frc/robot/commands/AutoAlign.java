// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.lang.annotation.Target;
import java.lang.reflect.Array;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Localization;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {
  Vision m_vision;
  CommandSwerveDrivetrain m_drivetrain;
  Localization m_localization;
   
    double m_xKp = .5;
    double m_yKp = .5;
    double m_yawKp = .5;


  /** Creates a new AutoAlign. */
  public AutoAlign(Vision vision, CommandSwerveDrivetrain drivetrain, Localization localization) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    m_vision = vision;
    m_drivetrain = drivetrain;
    m_localization = localization;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double vX = m_localization.getErrorX() * m_xKp;
    double vY = m_localization.getErrorY() * m_yKp;
    double vYaw = m_localization.getErrorYaw() * m_yawKp;
    
    m_drivetrain.Move(vX, vY, vYaw);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
