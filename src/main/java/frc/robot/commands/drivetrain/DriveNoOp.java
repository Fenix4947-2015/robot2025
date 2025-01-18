package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.Drivetrain;

public class DriveNoOp extends Command {

    //private final Frc4947Controller m_controller;
    private final Drivetrain m_driveTrain;

    public DriveNoOp(Drivetrain driveTrain) {
        m_driveTrain = driveTrain;
        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        driveWithJoystick(true);
        m_driveTrain.updateSmartDashboard();
    }

    private void driveWithJoystick(boolean fieldRelative) {
        m_driveTrain.driveNormalized(0, 0, 0, fieldRelative);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
