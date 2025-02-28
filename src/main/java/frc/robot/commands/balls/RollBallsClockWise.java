package frc.robot.commands.balls;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Balls;

public class RollBallsClockWise extends Command {
    private final Balls m_balls;

    public RollBallsClockWise(Balls balls) {
        m_balls = balls;
        addRequirements(balls);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double speed = -1;

        m_balls.roll(speed);
    }

    @Override
    public void end(boolean interrupted) {
        m_balls.roll(0.0);
    }
}
