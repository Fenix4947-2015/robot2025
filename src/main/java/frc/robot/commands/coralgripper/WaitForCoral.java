package frc.robot.commands.coralgripper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralGripper;

public class WaitForCoral extends Command {

    private final CoralGripper m_coralGripper;

    public WaitForCoral(CoralGripper coralGripper) {
        m_coralGripper = coralGripper;
    }

    @Override
    public void initialize() {
        m_coralGripper.openFrontGripper();
        m_coralGripper.openSideGripper();
    }

    @Override
    public boolean isFinished() {
        return m_coralGripper.isCoralLoaded();
    }
}
