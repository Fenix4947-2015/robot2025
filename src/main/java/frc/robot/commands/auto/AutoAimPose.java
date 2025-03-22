package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.SmartDashboardSettings;
import frc.robot.SmartDashboardWrapper;
import frc.robot.limelight.Limelight2025;
import frc.robot.limelight.LimelightMegaTagType;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAimPose extends AutoMoveStrategy {

    private final Limelight2025 _limelight;
    private Pose2d _currentTarget;
    private final Transform2d _initialTaget;
    private int _activeFiducialId;

    public AutoAimPose(
        CommandSwerveDrivetrain driveTrain,
        SmartDashboardSettings smartDashboardSettings,
        Limelight2025 limelight,
        Pose2d target) {
            super(driveTrain, smartDashboardSettings, target);
            _limelight = limelight;
            _currentTarget = updateRobotPosition();
            _initialTaget = new Transform2d(new Pose2d(), target);
            _activeFiducialId = limelight.getActiveFiducialId();
    }

    public AutoAimPose(
        CommandSwerveDrivetrain driveTrain, 
        SmartDashboardSettings smartDashboardSettings,
        Limelight2025 limelight,
        Pose2d target,
        long setpointDelayMs,
        Pose2d posTolerance) {
            super(driveTrain, smartDashboardSettings, target, setpointDelayMs, posTolerance);
            _limelight = limelight;
            _currentTarget = updateRobotPosition();
            _initialTaget = new Transform2d(new Pose2d(), target);
            _activeFiducialId = limelight.getActiveFiducialId();
    }
    
    @Override
    public void initialize() {
        super.initialize();
        _driveTrain.setLimelightMegaTagType(LimelightMegaTagType.MEGA_TAG);
        _activeFiducialId = _limelight.getActiveFiducialId();
        _driveTrain.setLimelightToUse(_limelight.getLimelightToUse());
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public Pose2d updateRobotPosition() {
        return _driveTrain.getState().Pose;
    }

    @Override
    public Pose2d updateDestination() {
        Transform2d closestFiducial = _limelight.getFiducialRobotRelative(LimelightMegaTagType.MEGA_TAG);
        if (closestFiducial == null || _limelight.getFiducialId() != _activeFiducialId) {
            return _currentTarget;
        }

        Transform2d targetPose = pose2dAsTransform2d(_driveTrain.getState().Pose).plus(closestFiducial);
        _currentTarget = transform2dAsPose2d(targetPose.plus(_initialTaget));
        return _currentTarget;
    }
}
