package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.SmartDashboardSettings;
import frc.robot.SmartDashboardWrapper;
import frc.robot.limelight.Limelight2025;
import frc.robot.limelight.LimelightMegaTagType;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAimTrajectoryPose extends AutoMoveTrajectoryStrategy {

    private final Limelight2025 _limelight;
    private Pose2d _currentTarget;
    private final Transform2d _initialTaget;
    private int _activeFiducialId;

    public AutoAimTrajectoryPose(
        CommandSwerveDrivetrain driveTrain,
        SmartDashboardSettings smartDashboardSettings,
        Limelight2025 limelight,
        Pose2d target) {
            super(driveTrain, smartDashboardSettings, target);
            updateTarget();
            _limelight = limelight;
            _currentTarget = new Pose2d();
            _initialTaget = new Transform2d(new Pose2d(), target);
            _activeFiducialId = limelight.getActiveFiducialId();
    }

    public AutoAimTrajectoryPose(
        CommandSwerveDrivetrain driveTrain, 
        SmartDashboardSettings smartDashboardSettings,
        Limelight2025 limelight,
        Pose2d target,
        long setpointDelayMs,
        Pose2d posTolerance) {
            super(driveTrain, smartDashboardSettings, target, posTolerance);
            _limelight = limelight;
            _currentTarget = new Pose2d();
            _initialTaget = new Transform2d(new Pose2d(), target);
            _activeFiducialId = limelight.getActiveFiducialId();
    }
    
    @Override
    public void initialize() {
        getDrivetrain().setLimelightMegaTagType(LimelightMegaTagType.MEGA_TAG);
        _activeFiducialId = _limelight.getActiveFiducialId();
        getDrivetrain().setLimelightToUse(_limelight.getLimelightToUse());
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
        Transform2d closestFiducial = _limelight.getClosestFiducial(LimelightMegaTagType.MEGA_TAG);
        if (closestFiducial == null || _limelight.getFiducialId() != _activeFiducialId) {
            return;
        }
        
        Pose2d currentPose = getDrivetrain().getState().Pose;
        Transform2d targetPose = pose2dAsTransform2d(currentPose).plus(closestFiducial);
        Pose2d newTarget = transform2dAsPose2d(targetPose.plus(_initialTaget));
        SmartDashboardWrapper.putNumber("closestFiducialX", closestFiducial.getX());
        SmartDashboardWrapper.putNumber("closestFiducialY", closestFiducial.getY());
        SmartDashboardWrapper.putNumber("closestFiducialRot", closestFiducial.getRotation().getDegrees());
        SmartDashboardWrapper.putNumber("currentPoseX", currentPose.getX());
        SmartDashboardWrapper.putNumber("currentPoseY", currentPose.getY());
        SmartDashboardWrapper.putNumber("currentPoseRot", currentPose.getRotation().getDegrees());
        SmartDashboardWrapper.putNumber("newTargetX", newTarget.getX());
        SmartDashboardWrapper.putNumber("newTargetY", newTarget.getY());
        SmartDashboardWrapper.putNumber("newTargetRot", newTarget.getRotation().getDegrees());
        setTarget(newTarget);
    }
}
