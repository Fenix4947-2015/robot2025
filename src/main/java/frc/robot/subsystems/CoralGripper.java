package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElectricConstants;
import frc.robot.SmartDashboardWrapper;
import edu.wpi.first.math.filter.Debouncer; // Import the Debouncer

public class CoralGripper extends SubsystemBase {

    private final Solenoid m_sideGripper = new Solenoid(
            ElectricConstants.kPneumaticHubCanId,
            PneumaticsModuleType.REVPH,
            ElectricConstants.kArmSideGripperChannel);
    private final Solenoid m_frontGripper = new Solenoid(
            ElectricConstants.kPneumaticHubCanId,
            PneumaticsModuleType.REVPH,
            ElectricConstants.kArmFrontGripperChannel);
    private final Solenoid m_pusher = new Solenoid(
            ElectricConstants.kPneumaticHubCanId,
            PneumaticsModuleType.REVPH,
            ElectricConstants.kArmPusherChannel);

    private final DigitalInput m_gripperProximitySensor = new DigitalInput(
            ElectricConstants.kArmGripperProximityChannel);
    private final DigitalInput m_sideGripperLimitSwitch = new DigitalInput(
            ElectricConstants.kArmSideGripperLimitSwitch);
    private final DigitalInput m_frontGripperLimitSwitch = new DigitalInput(
            ElectricConstants.kArmFrontGripperLimitSwitch);

    // Create a Debouncer with a 50ms debounce period (adjust as needed)
    private final Debouncer m_proxDebouncer = new Debouncer(0.5);

    public CoralGripper() {
    }

    @Override
    public void periodic() {
        SmartDashboardWrapper.putBoolean("CoralGripper / sideGripperOpen", isSideGripperOpen());
        SmartDashboardWrapper.putBoolean("FrontGripperOpen", isFrontGripperOpen());
        SmartDashboardWrapper.putBoolean("CoralGripper / coralLoaded", isCoralLoaded());
    }

    public void openSideGripper() {
        m_sideGripper.set(false);
    }

    public void closeSideGripper() {
        m_sideGripper.set(true);
    }

    public void toggleSideGripper() {
        m_sideGripper.toggle();
    }

    public void openFrontGripper() {
        m_frontGripper.set(true);
    }

    public void closeFrontGripper() {
        m_frontGripper.set(false);
    }

    public void toggleFrontGripper() {
        m_frontGripper.toggle();
    }

    public void openPusher() {
        m_pusher.set(false);
    }

    public void closePusher() {
        m_pusher.set(true);
    }

    public void togglePusher() {
        m_pusher.toggle();
    }

    public boolean isSideGripperOpen() {
        return m_sideGripperLimitSwitch.get();
    }

    public boolean isFrontGripperOpen() {
        return !m_frontGripperLimitSwitch.get();
    }

    // Use the debouncer to filter out noise from the proximity sensor.
    public boolean isCoralLoaded() {
        return m_proxDebouncer.calculate(m_gripperProximitySensor.get());
    }

}
