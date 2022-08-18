// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Turret;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.turretConstants;


  /** Creates a new TurretSubsystem. */
public class TurretSubsystem extends SubsystemBase {


  private static AnalogPotentiometer pot = new AnalogPotentiometer(turretConstants.analogPort, turretConstants.fullRange, turretConstants.offset);
  private static CANSparkMax turretMotor = new CANSparkMax(31, MotorType.kBrushed);
  /** Creates a new TurretSubsystem. */
  public TurretSubsystem() {}

  public void setMotorSpeed(double speed){
    turretMotor.set(speed);
  }

  public double getAngle(){
    return pot.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("turret angle maybe", getAngle());
  }


  
}
