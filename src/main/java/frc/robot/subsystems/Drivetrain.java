// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants.DriveConstants;

public class Drivetrain extends SubsystemBase {
  private final DifferentialDriveOdometry m_odometry;

  
  private static final Encoder m_leftEncoder = new Encoder(
    DriveConstants.leftEncoderPorts[0],
    DriveConstants.leftEncoderPorts[1],
    DriveConstants.leftEncoderReversed,
    CounterBase.EncodingType.k4X);
  private static final Encoder m_rightEncoder = new Encoder(
    DriveConstants.rightEncoderPorts[0],
    DriveConstants.rightEncoderPorts[1],
    DriveConstants.rightEncoderReversed,
    CounterBase.EncodingType.k4X); 
  private static final AHRS m_gyro = new AHRS(SPI.Port.kMXP);



   private static final CANSparkMax rightMotor1 = new CANSparkMax(DriveConstants.rightMotor1CAN, MotorType.kBrushed);
   private static final CANSparkMax rightMotor = new CANSparkMax(DriveConstants.rightMotorCAN, MotorType.kBrushed);
   private static final CANSparkMax leftMotor = new CANSparkMax(DriveConstants.leftMotorCAN, MotorType.kBrushed);
   private static final CANSparkMax leftMotor1 = new CANSparkMax(DriveConstants.leftMotor1CAN, MotorType.kBrushed);


   private static final MotorControllerGroup rightMotors = new MotorControllerGroup(rightMotor, rightMotor1);
   private static final MotorControllerGroup leftMotors = new MotorControllerGroup(leftMotor, leftMotor1);
   public final DifferentialDrive differentialDrive = new DifferentialDrive(leftMotors, rightMotors);
   
  

  public Drivetrain() {
       m_leftEncoder.setDistancePerPulse(1.0/2040.0 * 1.5*2.0*3.14);
       m_rightEncoder.setDistancePerPulse(1.0/2040.0 * 1.5*2.0*3.14);
       m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(
        m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());

    var translation = m_odometry.getPoseMeters().getTranslation();
    SmartDashboard.putNumber("robot x", translation.getX());
    SmartDashboard.putNumber("robot y", translation.getY());
        
    SmartDashboard.putNumber("encodersol", m_leftEncoder.getDistance());
    SmartDashboard.putNumber("encoderright", m_rightEncoder.getDistance());
    SmartDashboard.putNumber("angle", getHeading());
  }

  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360);
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

 
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  public void arcadeDrive(double fwd, double rot) {
    differentialDrive.arcadeDrive(rot, fwd);
  }
  public void tankdrive(double fwd1, double fwd2) {
    differentialDrive.tankDrive(fwd1, fwd2);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
    differentialDrive.feed();
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  public Encoder getLeftEncoder() {
    return m_leftEncoder;
  }

  public Encoder getRightEncoder() {
    return m_rightEncoder;
  }

  public void setMaxOutput(double maxOutput) {
    differentialDrive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  public void tankOrArcadeDrive(boolean tank, double tankFwd1, double tankFwd2, double arcadeFwd, double arcadeRot){
    if (tank){
      tankdrive(tankFwd1, tankFwd2);
    }else{
      arcadeDrive(arcadeFwd, arcadeRot);
    }
  }

}