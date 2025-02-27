package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class MoveArmDropPosition extends Command {
    private final Arm m_arm;
    private final Arm.DropPosition m_dropPosition;

    public MoveArmDropPosition(Arm arm, Arm.DropPosition dropPosition) {
        m_arm = arm;
        m_dropPosition = dropPosition;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        m_arm.setPidMode();
        m_arm.goToDropPosition(m_dropPosition);
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.setTargetPositionAsCurrent();
    }

    @Override
    public boolean isFinished() {
        return m_arm.atSetpoint();
    }
}
