package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class StopArm extends Command {
    private final Arm m_arm;

    public StopArm(Arm arm) {
        m_arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        m_arm.setDirectMode();
    }

    @Override
    public void execute() {
        m_arm.setDirectOutput(0);
    }
}
