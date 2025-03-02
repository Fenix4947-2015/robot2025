package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.SmartDashboardSettings;
import frc.robot.limelight.LimelightMegaTagType;

public class AutoMoveRelative extends AutoMoveStrategy {

    public AutoMoveRelative(
        CommandSwerveDrivetrain driveTrain, 
        SmartDashboardSettings smartDashboardSettings,
        Pose2d target) {
            super(driveTrain, smartDashboardSettings, target);
    }

    public AutoMoveRelative(
        CommandSwerveDrivetrain driveTrain, 
        SmartDashboardSettings smartDashboardSettings,
        Pose2d target,
        long setpointDelayMs,
        Pose2d posTolerance) {
            super(driveTrain, smartDashboardSettings, target, setpointDelayMs, posTolerance);
    }

    @Override
    public void initialize() {
        _driveTrain.setLimelightMegaTagType(LimelightMegaTagType.NONE);
        Pose2d robotPose = _driveTrain.getState().Pose;
        Pose2d targetPose = robotPose.plus(pose2dAsTransform2d(getTargetPose()));
        setTarget(targetPose);
    }
} 
