// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Timeout extends CommandBase {
  //this is just a timeout command that runs for (time) seconds

  
  /** Creates a new Timeout. */
  long startTime;
  long currentTime;
  long totalTime;
  public Timeout(long time) {
    totalTime = time;
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
    }
    else{
      return false;
    }
  }
}
