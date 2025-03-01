package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.SmartDashboardSettings;
import frc.robot.SmartDashboardWrapper;
import frc.robot.limelight.LimelightFour;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AutoAimPose extends AutoMoveStrategy {

    private final LimelightFour _limelight;
    private final Transform2d _initialTaget;

    public AutoAimPose(
        CommandSwerveDrivetrain driveTrain,
        SmartDashboardSettings smartDashboardSettings,
        LimelightFour limelight,
        Pose2d target) {
            super(driveTrain, smartDashboardSettings, target);
            _limelight = limelight;
            _initialTaget = new Transform2d(new Pose2d(), target);
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

    @Override
    public Pose2d updateDestination() {
        Pose2d closestFiducial = _limelight.getClosestFiducial();
        SmartDashboardWrapper.putNumber("fiducailX", closestFiducial.getX());
        SmartDashboardWrapper.putNumber("fiducailY", closestFiducial.getY());
        SmartDashboardWrapper.putNumber("fiducailROt", closestFiducial.getRotation().getDegrees());

        Transform2d targetPose = new Transform2d(_driveTrain.getState().Pose, closestFiducial.rotateBy(Rotation2d.k180deg)); ;
        return transform2dAsPose2d(targetPose.plus(_initialTaget));
    }
}
