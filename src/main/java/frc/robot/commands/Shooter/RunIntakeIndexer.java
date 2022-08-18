
package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter.Indexer;
import frc.robot.subsystems.Shooter.Intake;

public class RunIntakeIndexer extends CommandBase {
  private final double m_speed;
  private final Indexer indexerSubsystem;
  private final Intake intakeSubsystem;
  private static boolean m_finished;

  /** Creates a new RunShooter. */
  public RunIntakeIndexer(Indexer indexer, Intake intake, double speed, boolean go) {
    //timeBoolean false ise time fark etmiyor rastege sayı verilebilir
    m_speed = speed;
    m_finished = go;
    indexerSubsystem = indexer;
    intakeSubsystem = intake;
    addRequirements(indexerSubsystem);
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    indexerSubsystem.runIndexer(m_speed);
    intakeSubsystem.runIntake(m_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexerSubsystem.runIndexer(0);
    intakeSubsystem.runIntake(0);
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
