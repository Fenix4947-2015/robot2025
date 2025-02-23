package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class KeepArmInPosition extends Command {
    private final Arm m_arm;
    private double m_targetPosition;

    public KeepArmInPosition(Arm arm, double targetPosition) {
        m_arm = arm;
        m_targetPosition = targetPosition;
        addRequirements(arm);
    }

    public KeepArmInPosition(Arm arm) {
        this(arm, arm.getEncoderDistance());
    }

    public void setPosition(double targetPosition) {
        m_targetPosition = targetPosition;
    }

    public void setPositionAsCurrent() {
        m_targetPosition = m_arm.getEncoderDistance();
    }

    @Override
    public void initialize() {
        m_arm.setPidMode();
    }

    @Override
    public void execute() {
        m_arm.setTargetPosition(m_targetPosition);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
