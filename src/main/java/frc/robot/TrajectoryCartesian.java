package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import java.util.List;

public class TrajectoryCartesian {
    
    private Drivetrain m_robotDrive = new Drivetrain();
    public Trajectory exampleTrajectory;
    public RamseteCommand default_trajectory;
    public Trajectory randomTrajectory;
    public Trajectory reallyRandomTrajectory;

    public TrajectoryCartesian(Drivetrain drive){
        m_robotDrive = drive;


        var autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                    DriveConstants.ksVolts,
                    DriveConstants.kvVoltSecondsPerMeter,
                    DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics,
                10);

    
        TrajectoryConfig configforward =
            new TrajectoryConfig(
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);

        TrajectoryConfig configbackward =
                new TrajectoryConfig(
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);
        configbackward.setReversed(true);

        // An example trajectory to follow.  All units in meters.
        exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(0, -3, new Rotation2d(1)),
                // Pass config
                configforward);

        randomTrajectory = 
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0,0,new Rotation2d(0)), 
                List.of(), 
                new Pose2d(0, -3, new Rotation2d(0)), 
                configforward);

        reallyRandomTrajectory =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0,0,new Rotation2d(0)),
                List.of(),
                new Pose2d(1,1, new Rotation2d(2)), 
                configbackward);

            
        default_trajectory =
                new RamseteCommand(
                    exampleTrajectory,
                    m_robotDrive::getPose,
                    new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                    new SimpleMotorFeedforward(
                        DriveConstants.ksVolts,
                        DriveConstants.kvVoltSecondsPerMeter,
                        DriveConstants.kaVoltSecondsSquaredPerMeter),
                    DriveConstants.kDriveKinematics,
                    m_robotDrive::getWheelSpeeds,
                    new PIDController(DriveConstants.kPDriveVel, 0, 0),
                    new PIDController(DriveConstants.kPDriveVel, 0, 0),
                    // RamseteCommand passes volts to the callback
                    m_robotDrive::tankDriveVolts,
                    m_robotDrive);
    }

    public RamseteCommand getRamsete(Trajectory trajectory){
        
        return new RamseteCommand(
            trajectory,
            m_robotDrive::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            m_robotDrive::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            m_robotDrive::tankDriveVolts,
            m_robotDrive);
    }
}
