package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.SmartDashboardSettings;
import frc.robot.limelight.LimelightFour;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAimPose extends AutoMoveStrategy {

    private final LimelightFour _limelight;

    public AutoAimPose(
        CommandSwerveDrivetrain driveTrain, 
        LimelightFour limelight,
        SmartDashboardSettings smartDashboardSettings,
        Pose2d target) {
            super(driveTrain, smartDashboardSettings, target);
            _limelight = limelight;
    }
    
    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public Pose2d updateRobotPosition() {

        return _driveTrain.getState().Pose;
    }
}
