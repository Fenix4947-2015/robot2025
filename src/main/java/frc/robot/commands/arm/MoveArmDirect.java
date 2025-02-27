package frc.robot.commands.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;

public class MoveArmDirect extends Command {
    private final Arm m_arm;
    private final CommandXboxController m_controller;
    private final KeepArmInPosition m_keepArmInPositionCommand;

    public MoveArmDirect(Arm arm, CommandXboxController controller, KeepArmInPosition keepArmInPositionCommand) {
        m_arm = arm;
        m_controller = controller;
        m_keepArmInPositionCommand = keepArmInPositionCommand;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        m_arm.setDirectMode();
    }

    @Override
    public void execute() {
        double speed = -m_controller.getLeftY();

        m_arm.setDirectOutput(MathUtil.applyDeadband(speed, 0.1));
        m_arm.setTargetPositionAsCurrent();
    }

    @Override
    public void end(boolean isFinished) {
    }
}
