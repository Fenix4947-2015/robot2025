package frc.robot.commands.winch;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CageGripper;

public class RollCageGripper extends Command {
    private final CageGripper m_cageGripper;
    private final double m_speed;

    private static final double DEFAULT_SPEED = -0.5;

    public RollCageGripper(CageGripper cageGripper) {
        this(cageGripper, DEFAULT_SPEED);
    }

    public RollCageGripper(CageGripper cageGripper, double speed) {
        m_cageGripper = cageGripper;
        m_speed = speed;
        addRequirements(cageGripper);
    }

    @Override
    public void execute() {
        m_cageGripper.roll(m_speed);
    }

}
