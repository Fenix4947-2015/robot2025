package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * AutoRelativeTrajectoryStrategy extends AutoTrajectoryStrategy to move the robot relative to its current pose.
 * It accepts a relative offset (translation and rotation) and computes the target pose by applying that offset
 * to the current robot pose. The trajectory is then generated using the provided max speeds and accelerations.
 *
 * Note: This implementation assumes that key fields in AutoTrajectoryStrategy are declared as protected.
 */
public class AutoRelativeTrajectoryStrategy extends AutoTrajectoryStrategy {
    private final Pose2d relativeOffset;
    private final double maxLinearSpeed;
    private final double maxLinearAccel;
    private final double maxAngularSpeed;
    private final double maxAngularAccel;

    /**
     * Constructs an AutoRelativeTrajectoryStrategy.
     *
     * @param driveTrain       the drivetrain subsystem
     * @param relativeOffset   the desired relative movement (translation and rotation)
     * @param maxLinearSpeed   maximum linear speed (m/s)
     * @param maxLinearAccel   maximum linear acceleration (m/s²)
     * @param maxAngularSpeed  maximum angular speed (rad/s)
     * @param maxAngularAccel  maximum angular acceleration (rad/s²)
     */
    public AutoRelativeTrajectoryStrategy(CommandSwerveDrivetrain driveTrain,
                                          Pose2d relativeOffset,
                                          double maxLinearSpeed, double maxLinearAccel,
                                          double maxAngularSpeed, double maxAngularAccel) {
        // Call parent's constructor with dummy poses. These will be updated in initialize().
        super(driveTrain, new Pose2d(), new Pose2d(), maxLinearSpeed, maxLinearAccel, maxAngularSpeed, maxAngularAccel);
        this.relativeOffset = relativeOffset;
        this.maxLinearSpeed = maxLinearSpeed;
        this.maxLinearAccel = maxLinearAccel;
        this.maxAngularSpeed = maxAngularSpeed;
        this.maxAngularAccel = maxAngularAccel;
    }

    @Override
    public void initialize() {
        // Retrieve the current pose from the drivetrain.
        Pose2d currentPose = driveTrain.getState().Pose;
        // Compute the absolute target pose by applying the relative offset.
        Transform2d relativeTransform = new Transform2d(
                new Translation2d(relativeOffset.getX(), relativeOffset.getY()),
                relativeOffset.getRotation());
        Pose2d targetPose = currentPose.transformBy(relativeTransform);

        // Update the parent's fields with the actual start and target poses.
        this.startPose = currentPose;
        this.endPose = targetPose;
        // Recompute the trajectory with the correct parameters.
        this.trajectory = new UARMTrajectory(currentPose, targetPose,
                maxLinearSpeed, maxLinearAccel, maxAngularSpeed, maxAngularAccel);
        // Reset the start time using the FPGA timestamp.
        this.startTime = Timer.getFPGATimestamp();
    }
}
