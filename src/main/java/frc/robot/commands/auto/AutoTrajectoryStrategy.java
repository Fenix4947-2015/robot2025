package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.geometry.Pose2d;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.SmartDashboardWrapper;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * AutoTrajectoryStrategy generates and follows a trajectory from a start pose to an end pose
 * using a UARM (Uniformly Accelerated/Decelerated Motion) profile. This command uses the FPGA
 * timestamp for timing and a HolonomicDriveController to compute the required chassis speeds.
 */
public class AutoTrajectoryStrategy extends Command {
    protected CommandSwerveDrivetrain driveTrain;
    protected Pose2d startPose;
    protected Pose2d endPose;
    protected UARMTrajectory trajectory;
    protected double startTime;
    protected HolonomicDriveController controller;

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
        this.trajectory = new UARMTrajectory(startPose, endPose,
                                               maxLinearSpeed, maxLinearAccel,
                                               maxAngularSpeed, maxAngularAccel);

        // Initialize PID controllers for X and Y directions.
        PIDController xController = new PIDController(10.0, 0.0, 0.0);
        PIDController yController = new PIDController(10.0, 0.0, 0.0);
        // For rotation, use a ProfiledPIDController with continuous input.
        ProfiledPIDController thetaController = new ProfiledPIDController(7.0, 0.0, 0.0,
                new TrapezoidProfile.Constraints(2 * Math.PI, 2 * Math.PI));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // Create the HolonomicDriveController with the PID controllers.
        this.controller = new HolonomicDriveController(xController, yController, thetaController);

        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        // Use the FPGA timestamp for timing.
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        double currentTime = Timer.getFPGATimestamp();
        double elapsedTime = currentTime - startTime;

        // Sample the UARM trajectory to get the desired state at this time.
        UARMTrajectory.State state;
        if (elapsedTime >= trajectory.getDuration()) {
            state = trajectory.sample(trajectory.getDuration());
        } else {
            state = trajectory.sample(elapsedTime);
        }
        

        // Get the current robot pose from the drivetrain.
        Pose2d currentPose = driveTrain.getState().Pose;

        // Compute the desired chassis speeds using the HolonomicDriveController.
        ChassisSpeeds chassisSpeeds = controller.calculate(currentPose, state.pose, state.linearVelocity, state.pose.getRotation());

        // Log trajectory information.

        // Command the drivetrain with the calculated speeds.
        driveTrain.applyRequest(() ->
            new SwerveRequest.RobotCentric()
                .withVelocityX(chassisSpeeds.vxMetersPerSecond)
                .withVelocityY(chassisSpeeds.vyMetersPerSecond)
                .withRotationalRate(chassisSpeeds.omegaRadiansPerSecond)
        ).execute();
    }

    @Override
    public boolean isFinished() {        
        return controller.atReference();
    }

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
