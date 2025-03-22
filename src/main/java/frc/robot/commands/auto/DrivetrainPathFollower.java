package frc.robot.commands.auto;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.limelight.Limelight2025;
import frc.robot.limelight.LimelightMegaTagType;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.ArrayList;
import java.util.List;

public class DrivetrainPathFollower extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Pose2d approachPose;
    private final Pose2d targetPose;
    private final Limelight2025 limelight;
    private int activeFiducialId;
    private Command followPathCommand;

    public DrivetrainPathFollower(
        CommandSwerveDrivetrain drivetrain, 
        Pose2d approachPose,
        Pose2d targetPose,
        Limelight2025 limelight) {
        this.drivetrain = drivetrain;
        this.approachPose = approachPose;
        this.targetPose = targetPose;
        this.limelight = limelight;
        this.activeFiducialId = limelight.getActiveFiducialId();
        this.followPathCommand = Commands.none();
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        drivetrain.setLimelightMegaTagType(LimelightMegaTagType.MEGA_TAG);
        activeFiducialId = limelight.getActiveFiducialId();
        drivetrain.setLimelightToUse(limelight.getLimelightToUse());
        Pose2d currentPose = drivetrain.getState().Pose;

        Pose2d closestFiducialBlueRelative = limelight.getClosestFiducial().getPose2dfromBlue();
        if (closestFiducialBlueRelative == null || limelight.getFiducialId() != activeFiducialId) {
            return;
        }

        List<Pose2d> poses = new ArrayList<>();
        poses.add(currentPose);
        poses.add(closestFiducialBlueRelative.plus(new Transform2d(new Pose2d(), approachPose)));
        poses.add(closestFiducialBlueRelative.plus(new Transform2d(new Pose2d(), targetPose)));

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

        PathConstraints constraints = new PathConstraints(1.0, 3.0, 2 * Math.PI, 4 * Math.PI);

        PathPlannerPath generatedPath = new PathPlannerPath(
            waypoints,
            constraints,
            null,  
            new GoalEndState(0.0, targetPose.getRotation())
        );

        generatedPath.preventFlipping = true;

        followPathCommand = drivetrain.followPathCommand(generatedPath);
        followPathCommand.initialize();
    }

    @Override
    public void execute() {
        followPathCommand.execute();
    }

    @Override
    public boolean isFinished() {
        return followPathCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        followPathCommand.end(interrupted);
    }
}
