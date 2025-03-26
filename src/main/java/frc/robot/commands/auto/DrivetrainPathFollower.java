package frc.robot.commands.auto;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.SwerveDriveBrake;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.ArrayList;
import java.util.List;

public class DrivetrainPathFollower extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final Pose2d approachPose;
    private final Pose2d targetPose;
    private final Limelight2025 limelight;
    private int activeFiducialId;
    private Command followPathCommand;
    private double MAX_RATIO = 0.5;

        private DrivetrainPathFollower(Builder builder) {
            this.drivetrain = builder.drivetrain;
            this.approachPose = builder.approachPose;
            this.targetPose = builder.targetPose;
            this.limelight = builder.limelight;
            this.activeFiducialId = this.limelight.getActiveFiducialId();
            this.followPathCommand = Commands.none();
            addRequirements(drivetrain);
        }

        public static class Builder {
            private CommandSwerveDrivetrain drivetrain;
            private Pose2d approachPose;
            private Pose2d targetPose;
            private Limelight2025 limelight;
    
            public Builder drivetrain(CommandSwerveDrivetrain drivetrain) {
                this.drivetrain = drivetrain;
                return this;
            }
    
            public Builder approachPose(Pose2d approachPose) {
                this.approachPose = approachPose;
                return this;
            }
    
            public Builder targetPose(Pose2d targetPose) {
                this.targetPose = targetPose;
                return this;
            }
    
            public Builder limelight(Limelight2025 limelight) {
                this.limelight = limelight;
                return this;
            }
            
            public DrivetrainPathFollower build() {
                return new DrivetrainPathFollower(this);
            }
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

        Pose2d finalPose = closestFiducialBlueRelative.plus(new Transform2d(new Pose2d(), targetPose));

        List<Pose2d> poses = new ArrayList<>();
        poses.add(currentPose);
        if (approachPose != null) {
            poses.add(closestFiducialBlueRelative.plus(new Transform2d(new Pose2d(), approachPose)));
        }
        poses.add(finalPose);

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

        PathConstraints constraints = new PathConstraints(
            5.0 * MAX_RATIO, 
            8.5 * MAX_RATIO, 
            Degrees.of(550).in(Radians) * MAX_RATIO, 
            Degrees.of(1000).in(Radians) * MAX_RATIO);

        
        SwerveDriveState startingState = drivetrain.getState();
        double velocityMPS = Math.sqrt(
            Math.pow(startingState.Speeds.vxMetersPerSecond,2) + 
            Math.pow(startingState.Speeds.vyMetersPerSecond,2));
        IdealStartingState idealStartingState = new IdealStartingState(velocityMPS, startingState.Pose.getRotation());

        PathPlannerPath generatedPath = new PathPlannerPath(
            waypoints,
            constraints,
            idealStartingState,  
            new GoalEndState(0.0, finalPose.getRotation())
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
