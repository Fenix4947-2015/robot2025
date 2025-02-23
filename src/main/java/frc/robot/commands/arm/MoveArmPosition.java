package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class MoveArmPosition extends Command {
    private final Arm m_arm;
    private final double m_targetPosition;

    public MoveArmPosition(Arm arm, double targetPosition) {
        m_arm = arm;
        m_targetPosition = targetPosition;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        m_arm.setPidMode();
        m_arm.setTargetPosition(m_targetPosition);
    }

    @Override
    public boolean isFinished() {
        return m_arm.atSetpoint();
    }
}
