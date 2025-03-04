package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.SmartDashboardSettings;
import frc.robot.limelight.LimelightMegaTagType;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoTrajectoryPose extends AutoMoveTrajectoryStrategy {
    private final Transform2d _initialTaget;
    public AutoTrajectoryPose(
        CommandSwerveDrivetrain driveTrain,
        SmartDashboardSettings smartDashboardSettings,
        Pose2d target) {
            super(driveTrain, smartDashboardSettings, target);
            updateTarget();
            new Pose2d();
            _initialTaget = new Transform2d(new Pose2d(), target);
    }

    public AutoTrajectoryPose(
        CommandSwerveDrivetrain driveTrain, 
        SmartDashboardSettings smartDashboardSettings,
        Pose2d target,
        long setpointDelayMs,
        Pose2d posTolerance) {
            super(driveTrain, smartDashboardSettings, target, posTolerance);
            new Pose2d();
            _initialTaget = new Transform2d(new Pose2d(), target);
    }
    
    @Override
    public void initialize() {
        getDrivetrain().setLimelightMegaTagType(LimelightMegaTagType.MEGA_TAG);
        updateTarget();
        super.initialize();
    }



    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public void execute() {
        super.execute();
    }

    public void updateTarget() {
        Pose2d currentPose = getDrivetrain().getState().Pose;
        Pose2d newTarget = currentPose.plus(_initialTaget);
        setTarget(newTarget);
    }
}
