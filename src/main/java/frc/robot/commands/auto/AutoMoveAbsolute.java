package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.SmartDashboardSettings;

public class AutoMoveAbsolute extends AutoMoveStrategy {

    public AutoMoveAbsolute(
        CommandSwerveDrivetrain driveTrain, 
        SmartDashboardSettings smartDashboardSettings,
        Pose2d target) {
            super(driveTrain, smartDashboardSettings, target);
    }

        public AutoMoveAbsolute(
        CommandSwerveDrivetrain driveTrain, 
        SmartDashboardSettings smartDashboardSettings,
        Pose2d target,
        long setpointDelayMs,
        Pose2d posTolerance) {
            super(driveTrain, smartDashboardSettings, target, setpointDelayMs, posTolerance);
    }
}
