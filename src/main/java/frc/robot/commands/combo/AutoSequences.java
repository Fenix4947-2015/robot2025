package frc.robot.commands.combo;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Position;
import frc.robot.RobotContainer;
import frc.robot.commands.arm.MoveArmPosition;
import frc.robot.commands.auto.AutoMoveAbsolute;
import frc.robot.commands.coralgripper.CloseFrontGripper;
import frc.robot.commands.coralgripper.CloseSideGripper;
import frc.robot.commands.coralgripper.OpenFrontGripper;
import frc.robot.commands.coralgripper.OpenSideGripper;

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
            new OpenSideGripper(m_robotContainer.m_coralGripper)
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
        final Pose2d posTolerance = new Pose2d(0.3, 0.3, Rotation2d.fromDegrees(10.0));
        return new AutoMoveAbsolute(
                        m_robotContainer.drivetrain,
                        m_robotContainer.smartDashboardSettings,
                        position.getPositionForTeam(m_robotContainer.m_alliance),
                        posTolerance);
    }

    // AUTOS

}
