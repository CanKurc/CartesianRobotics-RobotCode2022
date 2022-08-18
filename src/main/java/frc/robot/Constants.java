// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants{
        public static final int leftMotorCAN = 2;
        public static final int leftMotor1CAN = 4;
        public static final int rightMotorCAN = 6;
        public static final int rightMotor1CAN = 8;
        public static final int intake = 2;

        public static final int[] rightEncoderPorts = {0, 1}; //KONTROL ET
        public static final int[] leftEncoderPorts = {2, 3}; //KONTROL ET
        public static final boolean rightEncoderReversed = false; //KONTROL ET
        public static final boolean leftEncoderReversed = false; //KONTROL ET

        public static final double motorSpeedConstant = 1.2;
        public static final double liftSpeedConstant = 0.5;




        //CHARACTERIZATION YAPILDIKTAN SONRA DEGISTIR
        public static final double ksVolts = 0.97;
        public static final double kvVoltSecondsPerMeter = 1.08;
        public static final double kaVoltSecondsSquaredPerMeter = 0.06;
        public static final double kPDriveVel = 0.08;

        public static final double kTrackwidthMeters = 0.56;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
		public static final double sensitivityright = 0;
    }

    public static final class AutoConstants{
        public static final double kMaxSpeedMetersPerSecond = 1.2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.6;
    
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
}
