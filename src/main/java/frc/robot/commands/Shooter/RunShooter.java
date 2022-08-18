// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter.Shooter;

public class RunShooter extends CommandBase {

  private final double m_speed;
  private final Shooter shooterSubsystem;
  private boolean m_finished = true;
  

  /** Creates a new RunShooter. */
  public RunShooter(Shooter shooter, double speed, boolean go) {
    //timeBoolean false ise time fark etmiyor rastege sayı verilebilir
    m_finished = go;
    m_speed = speed;
    shooterSubsystem = shooter;
    addRequirements(shooterSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.runShooter(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.runShooter(0);
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
