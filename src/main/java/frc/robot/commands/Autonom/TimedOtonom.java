// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonom;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class TimedOtonom extends CommandBase {
  private Drivetrain drive;
  private long startTime;
  private long currentTime;
  private long totalTime;
  /** Creates a new TimedOtonom. */
  public TimedOtonom(Drivetrain driveTrain, long time) {
    drive = driveTrain;
    totalTime = time;
    addRequirements(drive);
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
    //drive.arcadeDrive(-1, 0);
    currentTime = System.nanoTime();
  }    

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //drive.arcadeDrive(0, 0);
    System.out.println("ended!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if((currentTime - startTime) * 1000000000L < totalTime){
      return false;
    }
    else{
      //drive.arcadeDrive(0,0);
      return true;
    }
  }
}
