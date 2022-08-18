// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Lift;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lift extends SubsystemBase {

  /** Creates a new Lift. */
  private static final VictorSPX lift_motor1 = new VictorSPX(1);//sol
  private static final VictorSPX lift_motor2 = new VictorSPX(3);//sol
  private static final VictorSPX lift_motor3 = new VictorSPX(5);//sağ
  private static final VictorSPX lift_motor4 = new VictorSPX(7);//sağ
  public Lift() {
  

  }
  public void liftUp(double speed){
    lift_motor1.set(VictorSPXControlMode.PercentOutput, speed);
    lift_motor2.set(VictorSPXControlMode.PercentOutput, speed);
    lift_motor3.set(VictorSPXControlMode.PercentOutput, -speed);
    lift_motor4.set(VictorSPXControlMode.PercentOutput, -speed);


  }
  public void liftDown(double speed){
    lift_motor1.set(VictorSPXControlMode.PercentOutput, -speed);
    lift_motor2.set(VictorSPXControlMode.PercentOutput, -speed);
    lift_motor3.set(VictorSPXControlMode.PercentOutput, speed);
    lift_motor4.set(VictorSPXControlMode.PercentOutput, speed);

    
  }
  public void stop(){
    lift_motor1.set(VictorSPXControlMode.PercentOutput, 0);
    lift_motor2.set(VictorSPXControlMode.PercentOutput, 0);
    lift_motor3.set(VictorSPXControlMode.PercentOutput,0);
    lift_motor4.set(VictorSPXControlMode.PercentOutput, 0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
