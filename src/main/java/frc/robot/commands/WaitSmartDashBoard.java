package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.SmartDashboardSettings;

public class WaitSmartDashBoard extends WaitCommand {

    private final SmartDashboardSettings m_smartDashboardSettings;   
    private double waitDelay; 
    public WaitSmartDashBoard(SmartDashboardSettings smartDashboardSettings) {
        super(0);
        m_smartDashboardSettings = smartDashboardSettings;
        waitDelay = 0;
    }

    @Override
    public void initialize() {
        waitDelay = m_smartDashboardSettings.getWaitDelay();
        super.initialize();
    }

    @Override
    public boolean isFinished() {
      return m_timer.hasElapsed(waitDelay);
    }

}
