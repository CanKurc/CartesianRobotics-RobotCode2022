// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter.Indexer;

public class RunIndexerTime extends CommandBase {
  /** Creates a new RunShooterTime. */
  Indexer m_indexer;
  long totalTime;
  long startTime;
  long currentTime;
  float m_speed;
  public RunIndexerTime(Indexer indexer, long time, float speed) {
    m_indexer = indexer;
    totalTime = time;
    m_speed = speed;
    addRequirements(m_indexer);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.nanoTime();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_indexer.runIndexer(m_speed);
    currentTime = System.nanoTime();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(currentTime - startTime >= totalTime * 1000000000L){
      return true;
    }else{
      return false;
    }
  }
}
