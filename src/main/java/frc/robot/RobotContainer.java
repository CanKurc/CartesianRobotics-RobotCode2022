// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Lift.RunLiftDown;
import frc.robot.commands.Lift.RunLiftUp;
import frc.robot.commands.Shooter.RunIndexer;
import frc.robot.commands.Shooter.RunShooter;
import frc.robot.commands.shooterCommands.Ended;
import frc.robot.commands.shooterCommands.Timeout;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Lift.Lift;
import frc.robot.subsystems.Shooter.Indexer;
import frc.robot.subsystems.Shooter.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  //private final Shooter m_shooter = new Shooter();
  
  //private final Intake m_intake = new Intake();
  private final Drivetrain drivetrain = new Drivetrain();
  private final Lift liftSubsystem = new Lift();

  //Trajectory
  private final TrajectoryCartesian traj = new TrajectoryCartesian(drivetrain);

  //Joysticks
  Joystick m_rightJoystick = new Joystick(0);
  Joystick m_leftJoystick = new Joystick(1);
  private final Shooter m_shooter = new Shooter();
  private final Indexer m_indexer = new Indexer();

  //Other
  //public static String state = "drive";
  public boolean intake_state = false;
  private boolean driveState = true;



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    
    /**
    m_shooter.setDefaultCommand(
        new RunCommand(
          () -> m_shooter.runShooter(1 - m_rightJoystick.getRawAxis(3)), drivetrain));
    **/
    drivetrain.setDefaultCommand(
        new RunCommand(

          () -> drivetrain.tankOrArcadeDrive(driveState, m_leftJoystick.getY()/getDriveSpeed() ,-m_rightJoystick.getY()/getDriveSpeed(), m_rightJoystick.getY()/getDriveSpeed(), -m_rightJoystick.getX()/getDriveSpeed()), drivetrain));
          //UNCOMMENT IF TANK DRIVE XBOX CONTROLLER
          //() -> drivetrain.tankdrive(m_rightJo  ystick.getRawAxis(1)/1.2, m_rightJoystick.getRawAxis(3)/1.2),drivetrain));

          //UNCOMMENT IF IT IS TANK DRIVE DOUBLE JOYSTICK 
          //() -> drivetrain.tankdrive(m_leftJoystick.getY()/1.3, -m_rightJoystick.getY()/1.3),drivetrain));

          //UNCOMMENT IF IT IS ARCADE DRIVE
          //() -> drivetrain.arcadeDrive(m_rightJoystick.getY()/DriveConstants.motorSpeedConstant, -m_rightJoystick.getX()/DriveConstants.motorSpeedConstant), drivetrain));
  
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {
    
    
      
      //DRIVE BUTTON CONFIGURATION
      /** 
      new JoystickButton(m_rightJoystick, 1)
        .whileHeld(new RunIntakeIndexer(m_indexer, m_intake, 0.5, true))//SAĞ KONTROLCÜ BAŞ PARMAK ALTI BUTONU, Speed değişecek
        .whenReleased(new RunIntakeIndexer(m_indexer, m_intake, 0, false));
      */
    new JoystickButton(m_rightJoystick, 12)
        .whileHeld(new RunShooter(m_shooter ,0.5, true))//SAĞ KONTROLCÜ BAŞ PARMAK ALTI BUTONU, Speed değişecek
        .whenReleased(new RunShooter(m_shooter, 0, false));
    new JoystickButton(m_rightJoystick, 1)
        .whileHeld(new RunIndexer(m_indexer ,0.4, true))//SAĞ KONTROLCÜ BAŞ PARMAK ALTI BUTONU, Speed değişecek
        .whenReleased(new RunIndexer(m_indexer, 0, false));
    new JoystickButton(m_leftJoystick, 1)
        .whileHeld(new RunIndexer(m_indexer ,-0.25, true))//SAĞ KONTROLCÜ BAŞ PARMAK ALTI BUTONU, Speed değişecek
        .whenReleased(new RunIndexer(m_indexer, 0, false));
    new JoystickButton(m_rightJoystick, 2)
        .whenPressed(() -> changeDriveState());
    new JoystickButton(m_rightJoystick, 11)
      .whileHeld(new RunLiftUp(liftSubsystem, DriveConstants.liftSpeedConstant, true))
      .whenReleased(new RunLiftUp(liftSubsystem, 0, false));
    new JoystickButton(m_rightJoystick, 9)
      .whileHeld(new RunLiftDown(liftSubsystem, DriveConstants.liftSpeedConstant, true))
      .whenReleased(new RunLiftDown(liftSubsystem, 0, false));

    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    /*
    drivetrain.resetOdometry(traj.exampleTrajectory.getInitialPose());
    return traj.getRamsete(traj.exampleTrajectory).withTimeout(5).andThen(new RunIndexer(m_indexer, 0.6, true));
    */
    return (new Timeout(5)).andThen(new Ended());

    //return(new TimedOtonom(drivetrain, 1));
  }

   public void changeDriveState(){
    if (driveState){
      driveState = false;
    }else{
      driveState = true;
    }
  }
    public double getDriveSpeed(){
      double joyValue = 1 + m_leftJoystick.getRawAxis(3);
      if (joyValue <= 1){
        return 1;
      }else{
        return joyValue;
      }
    }
 
}
