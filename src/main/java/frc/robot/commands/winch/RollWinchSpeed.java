package frc.robot.commands.winch;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Winch;

public class RollWinchSpeed extends Command {
    private final Winch m_winch;
    private final double m_speed;

    public RollWinchSpeed(Winch winch, double speed) {
        m_winch = winch;
        m_speed = speed;
        addRequirements(winch);
    }

    @Override
    public void execute() {
        m_winch.roll(m_speed);
    }
    
}
