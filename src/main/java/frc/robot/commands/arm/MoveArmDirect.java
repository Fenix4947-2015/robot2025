package frc.robot.commands.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;

public class MoveArmDirect extends Command {
    private final Arm m_arm;
    private final CommandXboxController m_controller;
    private final CommandXboxController m_controllerDriver;

    public MoveArmDirect(Arm arm, CommandXboxController controller,CommandXboxController controllerDriver) {
        m_arm = arm;
        m_controller = controller;
        m_controllerDriver = controllerDriver;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        m_arm.setDirectMode();
    }

    @Override
    public void execute() {
        double speed_helper = -m_controller.getLeftY();
        double speed_driver =  m_controllerDriver.getRightTriggerAxis() - m_controllerDriver.getLeftTriggerAxis();
        double speed;

        if (Math.abs(speed_driver) > 0.1) {
            speed = speed_driver;
        } else {
            speed = speed_helper;
        }	

        double realSpeed = MathUtil.applyDeadband(speed, 0.1);

        if (Math.abs(realSpeed) > 0.0) {
            m_arm.setDirectMode();
            m_arm.setDirectOutput(realSpeed);
        } else if (m_arm.getArmMode() != Arm.ArmMode.PID){
            m_arm.setPidMode();
            m_arm.setTargetPositionAsCurrent();
        }
    }

    @Override
    public void end(boolean isFinished) {
    }
}
