// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter.Intake;

public class toggleIntake extends CommandBase {

  private final double m_speed;
  private final Intake intakeSubsystem;
  private final RobotContainer m_cont = new RobotContainer();
  private boolean intake_s = m_cont.intake_state;

  /** Creates a new RunShooter. */
  public toggleIntake(Intake intake, double speed) {
    //timeBoolean false ise time fark etmiyor rastege sayı verilebilir
    m_speed = speed;
    intakeSubsystem = intake;
    addRequirements(intakeSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake_s = !intake_s;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.runIntake(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.runIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !intake_s;
  }
}
