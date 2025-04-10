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
        //public static final int kArmMotor1CanId = 5;
        public static final int kArmMotor1CanId = 6;
        public static final int kArmMotor2CanId = 7;
        public static final int kArmMotor3CanId = 8;
        public static final int kArmCancoderCanId = 9;
        public static final int kArmLowLimitSwitchChannel = 8;
        public static final int kArmGripperProximityChannel = 4;
        public static final int kArmSideGripperLimitSwitch = 7;
        public static final int kArmFrontGripperLimitSwitch = 6;
        public static final int kArmExtenderChannel = 4;
        public static final int kArmSideGripperChannel = 15; //15; //0
        public static final int kArmFrontGripperChannel = 2;
        public static final int kArmPusherChannel = 11;

        public static final int kBallsCanId = 12;

        public static final int kWinchCageGripperChannel = 15;
        public static final int kWinchMotorOneChannel = 16;
        public static final int kWinchMotorTwoChannel = 17;
        public static final int kWinchEncoderChannel1 = 0;
        public static final int kWinchEncoderChannel2 = 1;

        public static final int kPigeon2CanId = 3;
        public static final int kPneumaticHubCanId = 2;
    }

    public static class Arm {
        public static final double kP = 7.5;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kS = 0.01;
        public static final double kG = 0.025;
        public static final double kV = 0;
        public static final double kToleranceDistance = 0.01;
        public static final double kLowestPosition = 0.215;
        public static final double kLimitOutputUntilPosition = 0.394;
        public static final double applyDownwardStabilizingPosition = 0.25;
        public static final double kHighestPosition = 1.25;
        public static final double kCoralL4Position = 1.09;
        public static final double kCoralL3Position = 0.755;
        public static final double kCoralL2Position = 0.650;
        public static final double lowestRetractedPosition = 0.640;
        public static final double gripperDangerZonePosition = 0.3;
        public static final double horizontalPosition = 0.752;
        public static final double encoderToArmGearRatio = 12.0 / 36.0;
        public static final double kDownwardStablizingOutput = -0.025;
    }

    public static class Limelight {
        public static final double offsetX = -0.46;
        public static final double offsetY = 0;
        public static final double offsetZ = 0.43;
        public static final double angleCamera = 60 - 15;

    }

    public static class Drivetrain {
        public static final double PATHPLANNER_TOLERANCE_X = 0.02;
        public static final double PATHPLANNER_TOLERANCE_Y = 0.02;
        public static final double PATHPLANNER_TOLERANCE_ANGLE = 0.02;
    }
}
