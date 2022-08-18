// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter.Indexer;

public class RunIndexer extends CommandBase {
  /** Creates a new RunIndexer. */
  
  private final double m_speed;
  private final Indexer m_indexer;
  private boolean m_finished = true;

  public RunIndexer(Indexer indexer, double speed, boolean go) {
    m_speed = speed;
    m_finished = go;
    m_indexer = indexer;
    addRequirements(m_indexer);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_indexer.runIndexer(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexer.runIndexer(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_finished){
      return false;
    }
    else{
      return true;
    }
  }
}
