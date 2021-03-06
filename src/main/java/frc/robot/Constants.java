// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public static final int leftMotor = 7;
        public static final int rightMotor = 5;
        public static final boolean kLeftInvert = false;
        public static final boolean kRightInvert = true;
        public static final double kMaxOutput = 0.5;
        public static final double kTurnOutput = 0.7;
        public static final double kDistancePerPulseFactor = 6 * Math.PI / (double)36.00;

    }
    public static final class OIConstants {
        public static final int kLeftX = 0;
        public static final int kLeftY = 1;
        public static final int kLeftTrigger = 2;
        public static final int kRightTrigger = 3;
        public static final int kRightX = 4;
        public static final int kRightY = 5;
        public static final int kDriverController1Port = 0;
    }
}
