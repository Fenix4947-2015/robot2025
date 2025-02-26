package frc.robot.commands.coralgripper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralGripper;

public class OpenSideGripper extends Command {

    private final CoralGripper m_coralGripper;

    public OpenSideGripper(CoralGripper coralGripper) {
        m_coralGripper = coralGripper;
    }

    @Override
    public void initialize() {
        m_coralGripper.openSideGripper();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
