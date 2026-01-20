// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.OptionalDouble;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAlign extends Command {
  Vision vision;
  CommandSwerveDrivetrain drivetrain;
  /** Creates a new AutoAlign. */
  public AutoAlign(Vision m_vision, CommandSwerveDrivetrain m_Drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    vision = m_vision;
    drivetrain = m_Drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    OptionalDouble dist = vision.getDistanceToTag(1);
    double speed = 0;
    double rotationRate = 0;
    if(dist.isPresent() && (dist.getAsDouble() <= .95 || dist.getAsDouble() >= 1.05) ){
      speed = (dist.getAsDouble() - 1) * 3;
    }
    
    OptionalDouble horizontalOffset = vision.getTxncForTag(1);
    if(horizontalOffset.isPresent() && (horizontalOffset.getAsDouble() <= -.08 || horizontalOffset.getAsDouble() >= .08)){
      rotationRate = -horizontalOffset.getAsDouble()  * .15;
    }
    drivetrain.Move(speed,0,rotationRate);
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
