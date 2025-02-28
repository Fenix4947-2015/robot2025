package frc.robot.commands.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;

public class MoveArmDirectDriverUp extends Command {
    private final Arm m_arm;
    private final KeepArmInPosition m_keepArmInPositionCommand;

    public MoveArmDirectDriverUp(Arm arm, KeepArmInPosition keepArmInPositionCommand) {
        m_arm = arm;
        m_keepArmInPositionCommand = keepArmInPositionCommand;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        m_arm.setDirectMode();
    }

    @Override
    public void execute() {
        double speed = 1;

        m_arm.setDirectOutput(speed);
        m_arm.setTargetPositionAsCurrent();
    }

    @Override
    public void end(boolean isFinished) {
        m_arm.setDirectOutput(0);
    }
}
