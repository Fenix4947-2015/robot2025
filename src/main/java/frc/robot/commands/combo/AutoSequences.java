package frc.robot.commands.combo;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
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

}
