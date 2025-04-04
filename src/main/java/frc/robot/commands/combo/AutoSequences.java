package frc.robot.commands.combo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Position;
import frc.robot.RobotContainer;
import frc.robot.commands.arm.MoveArmDropPosition;
import frc.robot.commands.arm.MoveArmPosition;
import frc.robot.commands.auto.AutoAimPose;
import frc.robot.commands.auto.AutoMoveAbsolute;
import frc.robot.commands.auto.DrivetrainPathFollower;
import frc.robot.commands.coralgripper.*;
import frc.robot.commands.drivetrain.NoSpeedDriveSwerveCommand;
import frc.robot.commands.limelight.FindTarget;
import frc.robot.commands.limelight.ResetTarget;
import frc.robot.limelight.Limelight2025;
import frc.robot.subsystems.Arm;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Second;

public class AutoSequences {

    private final RobotContainer m_robotContainer;

    public AutoSequences(RobotContainer robotContainer) {
        m_robotContainer = robotContainer;
    }

    public Command clampCoral() {
        return new SequentialCommandGroup(
                new OpenFrontGripper(m_robotContainer.m_coralGripper),
                new ClosePusher(m_robotContainer.m_coralGripper),
                new WaitCommand(0.5),
                new CloseFrontGripper(m_robotContainer.m_coralGripper)
        );
    }

    public Command waitAndClampCoral() {
        return new SequentialCommandGroup(
                new WaitForCoral(m_robotContainer.m_coralGripper).withTimeout(Second.of(3)),
                new OpenFrontGripper(m_robotContainer.m_coralGripper),
                new ClosePusher(m_robotContainer.m_coralGripper),
                new WaitCommand(0.5),
                new CloseFrontGripper(m_robotContainer.m_coralGripper)
        );
    }

    public Command gripCoral() {
        return new SequentialCommandGroup(
                new CloseFrontGripper(m_robotContainer.m_coralGripper),
                new WaitCommand(0.25)
        );
    }

    public Command freeCoral() {
        return new SequentialCommandGroup(
                new OpenFrontGripper(m_robotContainer.m_coralGripper),
                new OpenSideGripper(m_robotContainer.m_coralGripper),
                new OpenPusher(m_robotContainer.m_coralGripper)
        );
    }

    public Command findTargetOrStop(Limelight2025 limelight) {
        return new SequentialCommandGroup(
            new FindTarget(limelight, m_robotContainer.drivetrain).withTimeout(Milliseconds.of(100)),
            new NoSpeedDriveSwerveCommand(m_robotContainer.drivetrain, limelight),
            new FindTarget(limelight, m_robotContainer.drivetrain)
        );
    }

    public Command dropCoral() {
        return new SequentialCommandGroup(
                new OpenPusher(m_robotContainer.m_coralGripper),
                new CloseSideGripper(m_robotContainer.m_coralGripper),
                new WaitCommand(0.35),
                new OpenFrontGripper(m_robotContainer.m_coralGripper),
                new WaitCommand(0.4),
                new OpenSideGripper(m_robotContainer.m_coralGripper),
                new WaitCommand(0.1)
        );
    }

    public Command dropCoralSlow() {
        return new SequentialCommandGroup(
                new OpenPusher(m_robotContainer.m_coralGripper),
                new CloseSideGripper(m_robotContainer.m_coralGripper),
                new WaitCommand(0.35),
                new OpenFrontGripper(m_robotContainer.m_coralGripper),
                new WaitCommand(0.6),
                new OpenSideGripper(m_robotContainer.m_coralGripper),
                new WaitCommand(0.1)
        );
    }

    public Command autoDropCoralL3Right() {
        return new SequentialCommandGroup(
            new ResetTarget(m_robotContainer.limelightFour, m_robotContainer.drivetrain),
                new MoveArmDropPosition(m_robotContainer.m_arm, Arm.DropPosition.L3),
                new OpenSideGripper(m_robotContainer.m_coralGripper),
                findTargetOrStop(m_robotContainer.limelightFour),
                moveFiducialRelativeRough(Position.L3_APPROACH_RIGHT, m_robotContainer.limelightFour),
                new FindTarget(m_robotContainer.limelightFour, m_robotContainer.drivetrain),
                moveFiducialRelative(Position.CORAL_L3_RIGHT, m_robotContainer.limelightFour),
                dropCoralSlow(),
                moveFiducialRelativeRough(Position.L3_APPROACH_RIGHT, m_robotContainer.limelightFour),
                new InstantCommand(() -> m_robotContainer.m_arm.extendExtender(), m_robotContainer.m_arm),
                new ResetTarget(m_robotContainer.limelightFour, m_robotContainer.drivetrain)
        );
    }

    public Command autoDropCoralL3RightPP() {
        return new SequentialCommandGroup(
            new ResetTarget(m_robotContainer.limelightFour, m_robotContainer.drivetrain),
                new MoveArmDropPosition(m_robotContainer.m_arm, Arm.DropPosition.L3),
                new OpenSideGripper(m_robotContainer.m_coralGripper),
                findTargetOrStop(m_robotContainer.limelightFour),
                moveFiducialRelativePP(Position.L3_APPROACH_RIGHT, Position.CORAL_L3_RIGHT, m_robotContainer.limelightFour),
                moveFiducialRelative(Position.CORAL_L3_RIGHT, m_robotContainer.limelightFour),
                new ParallelDeadlineGroup(
                    dropCoralSlow(), 
                    moveFiducialRelative(Position.CORAL_L3_RIGHT, m_robotContainer.limelightFour)),
                moveFiducialRelativeRough(Position.L3_APPROACH_RIGHT, m_robotContainer.limelightFour),
                new InstantCommand(() -> m_robotContainer.m_arm.extendExtender(), m_robotContainer.m_arm),
                new ResetTarget(m_robotContainer.limelightFour, m_robotContainer.drivetrain)
        );
    }

    public Command autoDropCoralL4Right() {
        return new SequentialCommandGroup(
            new ResetTarget(m_robotContainer.limelightFour, m_robotContainer.drivetrain),
                new MoveArmDropPosition(m_robotContainer.m_arm, Arm.DropPosition.L4),
                new OpenSideGripper(m_robotContainer.m_coralGripper),
                findTargetOrStop(m_robotContainer.limelightFour),
                moveFiducialRelativeRough(Position.L4_APPROACH_RIGHT, m_robotContainer.limelightFour),
                new FindTarget(m_robotContainer.limelightFour, m_robotContainer.drivetrain),
                moveFiducialRelative(Position.CORAL_L4_RIGHT, m_robotContainer.limelightFour),
                dropCoral(),
                moveFiducialRelativeRough(Position.L4_APPROACH_RIGHT, m_robotContainer.limelightFour),
                new ResetTarget(m_robotContainer.limelightFour, m_robotContainer.drivetrain)
        );
    }

    public Command autoDropCoralL3Left() {
        return new SequentialCommandGroup(
                new ResetTarget(m_robotContainer.limelightFour, m_robotContainer.drivetrain),
                new MoveArmDropPosition(m_robotContainer.m_arm, Arm.DropPosition.L3),
                new OpenSideGripper(m_robotContainer.m_coralGripper),
                findTargetOrStop(m_robotContainer.limelightFour),
                moveFiducialRelativeRough(Position.L3_APPROACH_LEFT, m_robotContainer.limelightFour),
                new FindTarget(m_robotContainer.limelightFour, m_robotContainer.drivetrain),
                moveFiducialRelative(Position.CORAL_L3_LEFT, m_robotContainer.limelightFour),
                dropCoralSlow(),
                moveFiducialRelativeRough(Position.L3_APPROACH_LEFT, m_robotContainer.limelightFour),
                new InstantCommand(() -> m_robotContainer.m_arm.extendExtender(), m_robotContainer.m_arm),
                new ResetTarget(m_robotContainer.limelightFour, m_robotContainer.drivetrain)
        );
    }

    public Command autoDropCoralL3LeftPP() {
        return new SequentialCommandGroup(
            new ResetTarget(m_robotContainer.limelightFour, m_robotContainer.drivetrain),
                new MoveArmDropPosition(m_robotContainer.m_arm, Arm.DropPosition.L3),
                new OpenSideGripper(m_robotContainer.m_coralGripper),
                findTargetOrStop(m_robotContainer.limelightFour),
                moveFiducialRelativePP(Position.L3_APPROACH_LEFT, Position.CORAL_L3_LEFT, m_robotContainer.limelightFour),
                moveFiducialRelative(Position.CORAL_L3_LEFT, m_robotContainer.limelightFour),
                dropCoralSlow(),
                moveFiducialRelativeRough(Position.L3_APPROACH_LEFT, m_robotContainer.limelightFour),
                new InstantCommand(() -> m_robotContainer.m_arm.extendExtender(), m_robotContainer.m_arm),
                new ResetTarget(m_robotContainer.limelightFour, m_robotContainer.drivetrain)
        );
    }

    public Command autoDropCoralL4Left() {
        return new SequentialCommandGroup(
                new ResetTarget(m_robotContainer.limelightFour, m_robotContainer.drivetrain),
                new MoveArmDropPosition(m_robotContainer.m_arm, Arm.DropPosition.L4),
                new OpenSideGripper(m_robotContainer.m_coralGripper),
                findTargetOrStop(m_robotContainer.limelightFour),
                moveFiducialRelativeRough(Position.L4_APPROACH_LEFT, m_robotContainer.limelightFour),
                new FindTarget(m_robotContainer.limelightFour, m_robotContainer.drivetrain),
                moveFiducialRelative(Position.CORAL_L4_LEFT, m_robotContainer.limelightFour),
                dropCoral(),
                moveFiducialRelativeRough(Position.L4_APPROACH_LEFT, m_robotContainer.limelightFour),
                new ResetTarget(m_robotContainer.limelightFour, m_robotContainer.drivetrain)
        );
    }

    public Command autoDropCoralL4LeftPP() {
        return new SequentialCommandGroup(
            new ResetTarget(m_robotContainer.limelightFour, m_robotContainer.drivetrain),
            new MoveArmDropPosition(m_robotContainer.m_arm, Arm.DropPosition.L4),
            new OpenSideGripper(m_robotContainer.m_coralGripper),
            findTargetOrStop(m_robotContainer.limelightFour),
            moveFiducialRelativePP(Position.L4_APPROACH_LEFT, Position.CORAL_L4_LEFT, m_robotContainer.limelightFour),
            moveFiducialRelative(Position.CORAL_L4_LEFT, m_robotContainer.limelightFour),
            dropCoral(),
            moveFiducialRelativeRough(Position.L4_APPROACH_LEFT, m_robotContainer.limelightFour),
            new ResetTarget(m_robotContainer.limelightFour, m_robotContainer.drivetrain)
        );
    }

    public Command autoDropCoralL4RightPP() {
        return new SequentialCommandGroup(
            new ResetTarget(m_robotContainer.limelightFour, m_robotContainer.drivetrain),
            new MoveArmDropPosition(m_robotContainer.m_arm, Arm.DropPosition.L4),
            new OpenSideGripper(m_robotContainer.m_coralGripper),
            findTargetOrStop(m_robotContainer.limelightFour),
            moveFiducialRelativePP(Position.L4_APPROACH_RIGHT, Position.CORAL_L4_RIGHT, m_robotContainer.limelightFour),
            moveFiducialRelative(Position.CORAL_L4_RIGHT, m_robotContainer.limelightFour),
            new ParallelDeadlineGroup(
                dropCoral(), 
                moveFiducialRelative(Position.CORAL_L4_RIGHT, m_robotContainer.limelightFour)),
            moveFiducialRelativeRough(Position.L4_APPROACH_RIGHT, m_robotContainer.limelightFour),
            new ResetTarget(m_robotContainer.limelightFour, m_robotContainer.drivetrain)
        );
    }

    public Command autoMoveCoralL4Right() {
        return new SequentialCommandGroup(
                new MoveArmPosition(m_robotContainer.m_arm, Constants.Arm.kCoralL4Position),
                moveFiducialRelative(Position.CORAL_L4_RIGHT, m_robotContainer.limelightFour)
        );
    }

    public Command autoPickupCoralStation1PP() {
        return new SequentialCommandGroup(
                new ResetTarget(m_robotContainer.limelightThree, m_robotContainer.drivetrain),
                new MoveArmPosition(m_robotContainer.m_arm, Constants.Arm.kLowestPosition),
                new OpenSideGripper(m_robotContainer.m_coralGripper),
                new OpenPusher(m_robotContainer.m_coralGripper),
                new OpenFrontGripper(m_robotContainer.m_coralGripper),
                findTargetOrStop(m_robotContainer.limelightThree),
                moveFiducialRelativePP(Position.STATION_1_APPROACH, Position.STATION_1, m_robotContainer.limelightThree),
                new WaitForCoral(m_robotContainer.m_coralGripper).withTimeout(Second.of(3)),
                new ParallelCommandGroup(
                        clampCoral(),
                        moveFiducialRelativeRough(Position.STATION_1_APPROACH, m_robotContainer.limelightThree)
                ),
                new ResetTarget(m_robotContainer.limelightThree, m_robotContainer.drivetrain)
        );
    }

    public Command autoPickupCoralStation1() {
        return new SequentialCommandGroup(
            new ResetTarget(m_robotContainer.limelightThree, m_robotContainer.drivetrain),
                new MoveArmPosition(m_robotContainer.m_arm, Constants.Arm.kLowestPosition),
                new OpenSideGripper(m_robotContainer.m_coralGripper),
                new OpenFrontGripper(m_robotContainer.m_coralGripper),
                new OpenPusher(m_robotContainer.m_coralGripper),
                findTargetOrStop(m_robotContainer.limelightThree),
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

    public Command autoMoveCoralL4LeftPPTest() {
        return new SequentialCommandGroup(
            new FindTarget(m_robotContainer.limelightFour, m_robotContainer.drivetrain),
            moveFiducialRelativePP( 
                Position.L4_APPROACH_LEFT, 
                Position.CORAL_L4_LEFT, 
                m_robotContainer.limelightFour
                )
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
        return new DrivetrainPathFollower.Builder()
            .drivetrain(m_robotContainer.drivetrain)
            .approachPose(null)
            .targetPose(position.getPositionForTeam(Alliance.Blue))
            .limelight(limelight)
            .endSpeed(0.5)
            .build();
    }

    public Command moveFiducialRelativePP(Position approach, Position target, Limelight2025 limelight) {
        return new DrivetrainPathFollower.Builder()
            .drivetrain(m_robotContainer.drivetrain)
            .approachPose(approach.getPositionForTeam(Alliance.Blue))
            .targetPose(target.getPositionForTeam(Alliance.Blue))
            .limelight(limelight)
            .build();
    }

    // AUTOS

}
