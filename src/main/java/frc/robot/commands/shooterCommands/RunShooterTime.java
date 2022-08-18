// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter.Shooter;

public class RunShooterTime extends CommandBase {
  /** Creates a new RunShooterTime. */
  Shooter m_shooter;
  long totalTime;
  long startTime;
  long currentTime;
  float m_speed;
  public RunShooterTime(Shooter shooter, long time, float speed) {
    m_shooter = shooter;
    totalTime = time;
    m_speed = speed;
    addRequirements(m_shooter);
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
    m_shooter.runShooter(m_speed);
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
