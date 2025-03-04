package frc.robot.commands.combo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Position;
import frc.robot.RobotContainer;
import frc.robot.commands.arm.MoveArmDropPosition;
import frc.robot.commands.arm.MoveArmPosition;
import frc.robot.commands.auto.AutoAimPose;
import frc.robot.commands.auto.AutoAimTrajectoryPose;
import frc.robot.commands.auto.AutoMoveAbsolute;
import frc.robot.commands.auto.AutoTrajectoryPose;
import frc.robot.commands.coralgripper.*;
import frc.robot.commands.limelight.FindTarget;
import frc.robot.commands.limelight.ResetTarget;
import frc.robot.limelight.Limelight2025;
import frc.robot.subsystems.Arm;

import static edu.wpi.first.units.Units.Second;

public class AutoSequences {

    private final RobotContainer m_robotContainer;

    public AutoSequences(RobotContainer robotContainer) {
        m_robotContainer = robotContainer;
    }

    public Command clampCoral() {
        return new SequentialCommandGroup(
                new OpenFrontGripper(m_robotContainer.m_coralGripper),
                new CloseSideGripper(m_robotContainer.m_coralGripper),
                new WaitCommand(0.5),
                new CloseFrontGripper(m_robotContainer.m_coralGripper),
                new WaitCommand(0.25)
        );
    }

    public Command freeCoral() {
        return new SequentialCommandGroup(
                new OpenFrontGripper(m_robotContainer.m_coralGripper),
                new OpenSideGripper(m_robotContainer.m_coralGripper)
        );
    }

    public Command dropCoral() {
        return new SequentialCommandGroup(
                new CloseSideGripper(m_robotContainer.m_coralGripper),
                new WaitCommand(0.5),
                new OpenFrontGripper(m_robotContainer.m_coralGripper),
                new WaitCommand(0.5),
                new OpenSideGripper(m_robotContainer.m_coralGripper),
                new WaitCommand(0.25)
        );
    }

    public Command autoDropCoralRight() {
        if (m_robotContainer.m_arm.isArmRetracted()) {
            return autoDropCoralL3Right();
        }
        return autoDropCoralL4Right();
    }

    public Command autoDropCoralL3Right() {
        return new SequentialCommandGroup(
                new MoveArmDropPosition(m_robotContainer.m_arm, Arm.DropPosition.L3),
                new OpenSideGripper(m_robotContainer.m_coralGripper),
                new FindTarget(m_robotContainer.limelightFour, m_robotContainer.drivetrain),
                moveFiducialRelativeRough(Position.L3_APPROACH_RIGHT, m_robotContainer.limelightFour),
                new FindTarget(m_robotContainer.limelightFour, m_robotContainer.drivetrain),
                moveFiducialRelative(Position.CORAL_L3_RIGHT, m_robotContainer.limelightFour),
                dropCoral(),
                moveFiducialRelativeRough(Position.L3_APPROACH_RIGHT, m_robotContainer.limelightFour),
                new ResetTarget(m_robotContainer.limelightFour, m_robotContainer.drivetrain)
        );
    }

    public Command autoDropCoralL4Right() {
        return new SequentialCommandGroup(
                new MoveArmDropPosition(m_robotContainer.m_arm, Arm.DropPosition.L4),
                new OpenSideGripper(m_robotContainer.m_coralGripper),
                new FindTarget(m_robotContainer.limelightFour, m_robotContainer.drivetrain),
                moveFiducialRelativeRough(Position.L4_APPROACH_RIGHT, m_robotContainer.limelightFour),
                new FindTarget(m_robotContainer.limelightFour, m_robotContainer.drivetrain),
                moveFiducialRelative(Position.CORAL_L4_RIGHT, m_robotContainer.limelightFour),
                dropCoral(),
                moveFiducialRelativeRough(Position.L4_APPROACH_RIGHT, m_robotContainer.limelightFour),
                new ResetTarget(m_robotContainer.limelightFour, m_robotContainer.drivetrain)
        );
    }

    public Command autoDropCoralLeft() {
        if (m_robotContainer.m_arm.isArmRetracted()) {
            return autoDropCoralL3Left();
        }
        return autoDropCoralL4Left();
    }

    public Command autoDropCoralL3Left() {
        return new SequentialCommandGroup(
                new MoveArmDropPosition(m_robotContainer.m_arm, Arm.DropPosition.L3),
                new OpenSideGripper(m_robotContainer.m_coralGripper),
                new FindTarget(m_robotContainer.limelightFour, m_robotContainer.drivetrain),
                moveFiducialRelativeRough(Position.L3_APPROACH_LEFT, m_robotContainer.limelightFour),
                new FindTarget(m_robotContainer.limelightFour, m_robotContainer.drivetrain),
                moveFiducialRelative(Position.CORAL_L3_LEFT, m_robotContainer.limelightFour),
                dropCoral(),
                moveFiducialRelativeRough(Position.L3_APPROACH_LEFT, m_robotContainer.limelightFour),
                new ResetTarget(m_robotContainer.limelightFour, m_robotContainer.drivetrain)
        );
    }

    public Command autoDropCoralL4Left() {
        return new SequentialCommandGroup(
                new MoveArmDropPosition(m_robotContainer.m_arm, Arm.DropPosition.L4),
                new OpenSideGripper(m_robotContainer.m_coralGripper),
                new FindTarget(m_robotContainer.limelightFour, m_robotContainer.drivetrain),
                moveFiducialRelativeRough(Position.L4_APPROACH_LEFT, m_robotContainer.limelightFour),
                new FindTarget(m_robotContainer.limelightFour, m_robotContainer.drivetrain),
                moveFiducialRelative(Position.CORAL_L4_LEFT, m_robotContainer.limelightFour),
                dropCoral(),
                moveFiducialRelativeRough(Position.L4_APPROACH_LEFT, m_robotContainer.limelightFour),
                new ResetTarget(m_robotContainer.limelightFour, m_robotContainer.drivetrain)
        );
    }

    public Command autoMoveCoralL4Right() {
        return new SequentialCommandGroup(
                new MoveArmPosition(m_robotContainer.m_arm, Constants.Arm.kCoralL4Position),
                moveFiducialRelative(Position.CORAL_L4_RIGHT, m_robotContainer.limelightFour)
        );
    }

    public Command autoPickupCoralStation1() {
        return new SequentialCommandGroup(
                new MoveArmPosition(m_robotContainer.m_arm, Constants.Arm.kLowestPosition),
                new OpenSideGripper(m_robotContainer.m_coralGripper),
                new OpenFrontGripper(m_robotContainer.m_coralGripper),
                new FindTarget(m_robotContainer.limelightThree, m_robotContainer.drivetrain),
                moveFiducialRelativeRough(Position.STATION_1_APPROACH, m_robotContainer.limelightThree),
                moveFiducialRelativeRough(Position.STATION_1, m_robotContainer.limelightThree),
                new WaitForCoral(m_robotContainer.m_coralGripper).withTimeout(Second.of(3)),
                new ParallelCommandGroup(
                        clampCoral(),
                        moveFiducialRelativeRough(Position.STATION_1_APPROACH, m_robotContainer.limelightThree)
                ),
                new ResetTarget(m_robotContainer.limelightThree, m_robotContainer.drivetrain)
        );
    }

    public Command autoDropTrajerctoryCoralL4Left() {
        return new SequentialCommandGroup(
                new MoveArmPosition(m_robotContainer.m_arm, Constants.Arm.kCoralL4Position),
                new OpenSideGripper(m_robotContainer.m_coralGripper),
                new FindTarget(m_robotContainer.limelightFour, m_robotContainer.drivetrain),
                moveTrajerctoryFiducialRelativeRough(Position.L4_APPROACH_LEFT, m_robotContainer.limelightFour),
                new ResetTarget(m_robotContainer.limelightFour, m_robotContainer.drivetrain)
        );
    }

    public Command auto1m() {
        return new SequentialCommandGroup(
                moveTrajerctoryRelativeRough(new Pose2d(1, 0, Rotation2d.fromDegrees(90)))
        );
    }

    public Command armToLowestPosition() {
        return new MoveArmPosition(m_robotContainer.m_arm, Constants.Arm.kLowestPosition);
    }

    public Command armToAmpPosition() {
        return new MoveArmPosition(m_robotContainer.m_arm, Constants.Arm.kHighestPosition);
    }

    public Command armToSafePosition() {
        return new MoveArmPosition(m_robotContainer.m_arm, Constants.Arm.kLowestPosition);
    }

    public Command moveAbsolute(Position position) {
        return new AutoMoveAbsolute(
                m_robotContainer.drivetrain,
                m_robotContainer.smartDashboardSettings,
                position.getPositionForTeam(m_robotContainer.m_alliance));
    }

    public Command moveAbsoluteRough(Position position) {
        final Pose2d posTolerance = new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(3));
        return new AutoMoveAbsolute(
                m_robotContainer.drivetrain,
                m_robotContainer.smartDashboardSettings,
                position.getPositionForTeam(m_robotContainer.m_alliance),
                0,
                posTolerance);
    }

    public Command moveFiducialRelative(Position position, Limelight2025 limelight) {
        return new AutoAimPose(
                m_robotContainer.drivetrain,
                m_robotContainer.smartDashboardSettings,
                limelight,
                position.getPositionForTeam(m_robotContainer.m_alliance)
        );
    }

    public Command moveFiducialRelativeRough(Position position, Limelight2025 limelight) {
        final Pose2d posTolerance = new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(3));
        return new AutoAimPose(
                m_robotContainer.drivetrain,
                m_robotContainer.smartDashboardSettings,
                limelight,
                position.getPositionForTeam(m_robotContainer.m_alliance),
                0,
                posTolerance);
    }

    public Command moveTrajerctoryFiducialRelativeRough(Position position, Limelight2025 limelight) {
        final Pose2d posTolerance = new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(3));
        return new AutoAimTrajectoryPose(
                m_robotContainer.drivetrain,
                m_robotContainer.smartDashboardSettings,
                limelight,
                position.getPositionForTeam(m_robotContainer.m_alliance),
                0,
                posTolerance);
    }

    public Command moveTrajerctoryRelativeRough(Pose2d pose) {
        final Pose2d posTolerance = new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(3));
        return new AutoTrajectoryPose(
                m_robotContainer.drivetrain,
                m_robotContainer.smartDashboardSettings,
                pose,
                0,
                posTolerance);
    }

    // AUTOS

}
