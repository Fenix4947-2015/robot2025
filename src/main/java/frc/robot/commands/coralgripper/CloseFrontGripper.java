package frc.robot.commands.coralgripper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralGripper;

public class CloseFrontGripper extends Command {

    private final CoralGripper m_coralGripper;

    public CloseFrontGripper(CoralGripper coralGripper) {
        m_coralGripper = coralGripper;
    }

    @Override
    public void initialize() {
        m_coralGripper.closeFrontGripper();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
