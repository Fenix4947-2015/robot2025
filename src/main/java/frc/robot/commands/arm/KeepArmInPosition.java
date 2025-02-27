package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class KeepArmInPosition extends Command {
    private final Arm m_arm;

    public KeepArmInPosition(Arm arm) {
        m_arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        m_arm.setPidMode();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
