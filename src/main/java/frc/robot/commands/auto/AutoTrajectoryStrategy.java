package frc.robot.commands.auto;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SmartDashboardWrapper;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * AutoTrajectoryStrategy generates and follows a trajectory from a start pose to an end pose.
 * The trajectory is generated using a UARM (Uniformly Accelerated/Decelerated Motion) profile
 * for both translation and rotation. The command samples the trajectory based on elapsed time
 * and commands the drivetrain with the desired linear and angular velocities.
 */
public class AutoTrajectoryStrategy extends Command {
    private final CommandSwerveDrivetrain driveTrain;
    private final Pose2d startPose;
    private final Pose2d endPose;
    private final UARMTrajectory trajectory;
    private long startTimeMs;

    /**
     * Constructs an AutoTrajectoryStrategy.
     *
     * @param driveTrain       the drivetrain subsystem to command
     * @param startPose        the starting pose of the trajectory
     * @param endPose          the ending pose of the trajectory
     * @param maxLinearSpeed   maximum linear speed (m/s)
     * @param maxLinearAccel   maximum linear acceleration (m/s^2)
     * @param maxAngularSpeed  maximum angular speed (rad/s)
     * @param maxAngularAccel  maximum angular acceleration (rad/s^2)
     */
    public AutoTrajectoryStrategy(CommandSwerveDrivetrain driveTrain,
                                  Pose2d startPose,
                                  Pose2d endPose,
                                  double maxLinearSpeed, double maxLinearAccel,
                                  double maxAngularSpeed, double maxAngularAccel) {
        this.driveTrain = driveTrain;
        this.startPose = startPose;
        this.endPose = endPose;
        this.trajectory = new UARMTrajectory(startPose, endPose, maxLinearSpeed, maxLinearAccel, maxAngularSpeed, maxAngularAccel);
        addRequirements(driveTrain);
    }

    // Called once when the command is initially scheduled.
    @Override
    public void initialize() {
        startTimeMs = System.currentTimeMillis();
    }

    // Called repeatedly while the command is scheduled.
    @Override
    public void execute() {
        double elapsedSec = (System.currentTimeMillis() - startTimeMs) / 1000.0;
        UARMTrajectory.State state = trajectory.sample(elapsedSec);

        // Decompose the linear velocity along the vector from startPose to endPose.
        double dx = endPose.getX() - startPose.getX();
        double dy = endPose.getY() - startPose.getY();
        double norm = Math.hypot(dx, dy);
        double unitX = (norm > 0) ? dx / norm : 0;
        double unitY = (norm > 0) ? dy / norm : 0;
        double commandedVx = state.linearVelocity * unitX;
        double commandedVy = state.linearVelocity * unitY;

        // Log trajectory information to the dashboard.
        SmartDashboardWrapper.putNumber("TrajectoryTime", elapsedSec);
        SmartDashboardWrapper.putNumber("TrajectoryX", state.pose.getX());
        SmartDashboardWrapper.putNumber("TrajectoryY", state.pose.getY());
        SmartDashboardWrapper.putNumber("TrajectoryRot", state.pose.getRotation().getDegrees());
        SmartDashboardWrapper.putNumber("TrajectoryLinearVel", state.linearVelocity);
        SmartDashboardWrapper.putNumber("TrajectoryAngularVel", state.angularVelocity);

        // Apply the trajectory commands to the drivetrain.
        driveTrain.applyRequest(() ->
            new SwerveRequest.RobotCentric()
                .withVelocityX(commandedVx)
                .withVelocityY(commandedVy)
                .withRotationalRate(state.angularVelocity)
        ).execute();
    }

    // Returns true when the trajectory is complete.
    @Override
    public boolean isFinished() {
        double elapsedSec = (System.currentTimeMillis() - startTimeMs) / 1000.0;
        return elapsedSec >= trajectory.getDuration();
    }

    // Called once after isFinished returns true.
    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain.
        driveTrain.applyRequest(() ->
            new SwerveRequest.RobotCentric()
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        ).execute();
    }
}

