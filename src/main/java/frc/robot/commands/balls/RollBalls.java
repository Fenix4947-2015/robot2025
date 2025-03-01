package frc.robot.commands.balls;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Balls;

public class RollBalls extends Command {
    private final Balls m_balls;
    private final CommandXboxController m_controller;

    public RollBalls(Balls balls, CommandXboxController controller) {
        m_balls = balls;
        m_controller = controller;
        addRequirements(balls);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double speed = m_controller.getRightTriggerAxis() - m_controller.getLeftTriggerAxis();
        speed = MathUtil.clamp(speed, -1.0, 1.0);
        m_balls.roll(speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_balls.roll(0.0);
    }
}
