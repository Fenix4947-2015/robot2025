package frc.robot.commands.combo;

import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Position;
import frc.robot.RobotContainer;
import frc.robot.commands.arm.MoveArmPosition;
import frc.robot.commands.auto.AutoAimPose;
import frc.robot.commands.auto.AutoMoveAbsolute;
import frc.robot.commands.auto.AutoMoveRelative;
import frc.robot.commands.coralgripper.CloseFrontGripper;
import frc.robot.commands.coralgripper.CloseSideGripper;
import frc.robot.commands.coralgripper.OpenFrontGripper;
import frc.robot.commands.coralgripper.OpenSideGripper;
import frc.robot.commands.coralgripper.WaitForCoral;
import frc.robot.commands.limelight.FindTarget;
import frc.robot.commands.limelight.ResetTarget;
import frc.robot.limelight.Limelight2025;

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

    public Command autoDropCoralL4Right() {
        return new SequentialCommandGroup(
            new MoveArmPosition(m_robotContainer.m_arm, Constants.Arm.kCoralL4Position),
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

    public Command autoDropCoralL4Left() {
        return new SequentialCommandGroup(
            new MoveArmPosition(m_robotContainer.m_arm, Constants.Arm.kCoralL4Position),
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
            clampCoral(),
            moveFiducialRelativeRough(Position.STATION_1_APPROACH, m_robotContainer.limelightThree),
            new ResetTarget(m_robotContainer.limelightThree, m_robotContainer.drivetrain)
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
        final Pose2d posTolerance = new Pose2d(0.1,0.1, Rotation2d.fromDegrees(3));
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
        final Pose2d posTolerance = new Pose2d(0.1,0.1, Rotation2d.fromDegrees(3));
        return new AutoAimPose(
                        m_robotContainer.drivetrain,
                        m_robotContainer.smartDashboardSettings,
                        limelight,
                        position.getPositionForTeam(m_robotContainer.m_alliance),
                        0,
                        posTolerance);
    }

    public Command moveRelativeRough(Pose2d pose2d) {
        final Pose2d posTolerance = new Pose2d(0.1,0.1, Rotation2d.fromDegrees(3));
        return new AutoMoveRelative(
                        m_robotContainer.drivetrain,
                        m_robotContainer.smartDashboardSettings,
                        pose2d,
                        0,
                        posTolerance);
    }

    // AUTOS

}
