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
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kHelperControllerPort = 1;
    }

    public static class ElectricConstants {
        public static final int kIntakeMotorChannel = 35;
        public static final int kIntakeMotorThirdlinkChannel = 28;
        public static final int kIntakeDetectorChannel = 5;
        public static final int kShooterMotorTopChannel = 27;
        public static final int kShooterMotorBottomChannel = 23;

        public static final int kArmMotor1CanId = 5;
        public static final int kArmMotor2CanId = 6;
        public static final int kArmMotor3CanId = 7;
        public static final int kArmMotor4CanId = 8;
        public static final int kArmCancoderCanId = 9;
        public static final int kArmLowLimitSwitchChannel = 8;

        public static final int kArmEncoderChannel1 = 7;
        public static final int kArmEncoderChannel2 = 6;
        public static final int kWinchMotorOneChannel = 24;
        public static final int kWinchMotorTwoChannel = 29;
        public static final int kWinchEncoderChannel1 = 9;
        public static final int kWinchEncoderChannel2 = 8;
        public static final int kWinchSafetySwitchChannel = 6;

        public static final int kPigeon2Channel = 3;
    }

    public static class Arm {
        public static final double kP = 0.06;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0.06;
        public static final double kV = 0;
        public static final double kToleranceDegrees = 1.0;
        public static final double kLowestPosition = 100.0;
        public static final double kHighestPosition = 5.0;
        public static final double kSafePosition = 50.0;
        public static final double kDistanceShootNear = 1.5;
        public static final double kDistanceShootFar = 3.4;
        public static final double kAngleShootNear = 100;
        public static final double kAngleShootFar = 75;
    }

    public static class Limelight {
        public static final double offsetX = -0.46;
        public static final double offsetY = 0;
        public static final double offsetZ = 0.43;
        public static final double angleCamera = 60 - 15;

    }
}
