// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  private static final CANSparkMax indexer = new CANSparkMax(10, MotorType.kBrushed);
  /** Creates a new Indexer. */
  public Indexer() {}

  public void runIndexer(double speed){
    indexer.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
