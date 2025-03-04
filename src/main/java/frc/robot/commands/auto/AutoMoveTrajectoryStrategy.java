package frc.robot.commands.auto;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.SmartDashboardSettings;
import frc.robot.SmartDashboardWrapper;

import java.util.List;

import static edu.wpi.first.units.Units.*;

public class AutoMoveTrajectoryStrategy extends Command {
    // PID constants for trajectory following (tweak these for your system)
    public static final double K_PID_P_DISTANCE = 10;
    public static final double K_PID_I_DISTANCE = 0.0;
    public static final double K_PID_D_DISTANCE = 0.0;
    public static final double K_PID_P_ANGLE = 7;
    public static final double K_PID_I_ANGLE = 0.0;
    public static final double K_PID_D_ANGLE = 0.0;

    // Tolerance for considering the motion “complete”
    private final Pose2d _posTolerance;
    public static final Pose2d DEFAULT_POS_TOLERANCE = new Pose2d(0.05, 0.05, Rotation2d.fromDegrees(0.5));

    private final CommandSwerveDrivetrain _driveTrain;
    private Pose2d _target;

    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) / 4;
    private final double MAX_LINEAR_ACCELERATION = 2; // 8.5m/s²
    private final double MAX_ANGULAR_SPEED = Math.toRadians(150); //578
    private final double MAX_ANGULAR_ACCELERATION = Math.toRadians(300); //1368

    // Trajectory and controller objects
    private Trajectory trajectory;
    private HolonomicDriveController holonomicController;
    private double _startTime;

    public AutoMoveTrajectoryStrategy(
            CommandSwerveDrivetrain driveTrain, 
            SmartDashboardSettings smartDashboardSettings,
            Pose2d target) {
        this(driveTrain, smartDashboardSettings, target, DEFAULT_POS_TOLERANCE);
    }

    public AutoMoveTrajectoryStrategy(
            CommandSwerveDrivetrain driveTrain, 
            SmartDashboardSettings smartDashboardSettings,
            Pose2d target,
            Pose2d posTolerance) {
        _driveTrain = driveTrain;
        _target = target;
        _posTolerance = posTolerance;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        // Get current pose from the drivetrain.
        Pose2d startPose = _driveTrain.getState().Pose;

        // Create a trajectory configuration with our given max speed and acceleration.
        TrajectoryConfig config = new TrajectoryConfig(MaxSpeed, MAX_LINEAR_ACCELERATION);
        // If you have your swerve kinematics, set them so the trajectory respects your robot's geometry.
        config.setKinematics(_driveTrain.getKinematics());

        SmartDashboardWrapper.putNumber("startPoseX", startPose.getX());
        SmartDashboardWrapper.putNumber("startPoseY", startPose.getY());
        SmartDashboardWrapper.putNumber("startPoseRot", startPose.getRotation().getDegrees());
        SmartDashboardWrapper.putNumber("_targetX", _target.getX());
        SmartDashboardWrapper.putNumber("_targetY", _target.getY());
        SmartDashboardWrapper.putNumber("_targetRot", _target.getRotation().getDegrees());

        // Generate a trajectory from the current pose to the target.
        trajectory = TrajectoryGenerator.generateTrajectory(
                startPose,       // Start at the current pose
                List.of(),       // No interior waypoints (straight line path)
                _target,         // End at the target pose
                config);

        // Initialize the HolonomicDriveController with PID controllers.
        holonomicController = new HolonomicDriveController(
                new PIDController(K_PID_P_DISTANCE, K_PID_I_DISTANCE, K_PID_D_DISTANCE),
                new PIDController(K_PID_P_DISTANCE, K_PID_I_DISTANCE, K_PID_D_DISTANCE),
                new ProfiledPIDController(
                    K_PID_P_ANGLE, 
                    K_PID_I_ANGLE, 
                    K_PID_D_ANGLE, 
                    new TrapezoidProfile.Constraints(MAX_ANGULAR_SPEED, MAX_ANGULAR_ACCELERATION)));
        // Set the tolerance (for example, to consider the robot at the target when errors are below these values)
        holonomicController.setTolerance(new Pose2d(
                _posTolerance.getX(), 
                _posTolerance.getY(), 
                _posTolerance.getRotation()));
        holonomicController.getThetaController().enableContinuousInput(-Math.PI, Math.PI);

        // Reset the timer.
        _startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        // Compute elapsed time since trajectory started.
        double elapsedTime = Timer.getFPGATimestamp() - _startTime;

        // Sample the desired state from the trajectory.
        Trajectory.State desiredState;
        if (elapsedTime > trajectory.getTotalTimeSeconds()) {
            // If we’re past the trajectory time, hold the final state.
            desiredState = trajectory.sample(trajectory.getTotalTimeSeconds());
        } else {
            desiredState = trajectory.sample(elapsedTime);
        }

        SmartDashboardWrapper.putNumber("desiredX", desiredState.poseMeters.getX());
        SmartDashboardWrapper.putNumber("desiredY", desiredState.poseMeters.getY());
        SmartDashboardWrapper.putNumber("desiredRot", desiredState.poseMeters.getRotation().getDegrees());

        // Use the trajectory’s heading for orientation.
        Rotation2d desiredRotation = desiredState.poseMeters.getRotation();

        // Get the current pose of the robot.
        Pose2d currentPose = _driveTrain.getState().Pose;

        // Compute the chassis speeds required to follow the trajectory.
        ChassisSpeeds chassisSpeeds = holonomicController.calculate(currentPose, desiredState, desiredRotation);

        // Normalize the speeds:
        // For translation, divide by MaxSpeed.
        double normX = chassisSpeeds.vxMetersPerSecond;
        double normY = chassisSpeeds.vyMetersPerSecond;
        // For rotation, use the provided maximum angular speed.
        double normOmega = chassisSpeeds.omegaRadiansPerSecond;

        // Send the computed speeds to the drivetrain.
        drive(normX, normY, normOmega);
    }

    private void drive(double x, double y, double steer) {
        // Use your drivetrain's method to apply the command.
        _driveTrain.applyRequest(() ->
                // Convert normalized values back to velocities using our max speed constants.
                new SwerveRequest.FieldCentric()
                        .withVelocityX(x)
                        .withVelocityY(y)
                        .withRotationalRate(steer)
        ).execute();
    }

    @Override
    public boolean isFinished() {
        // Consider the command finished if the trajectory time has elapsed and the robot is within tolerance.
        double elapsedTime = Timer.getFPGATimestamp() - _startTime;
        boolean trajectoryFinished = elapsedTime > trajectory.getTotalTimeSeconds();
        boolean atSetpoint = holonomicController.atReference();
        return trajectoryFinished && atSetpoint;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain when the command ends.
        drive(0, 0, 0);
    }

    // Optionally, if you wish to update the target mid-run:
    public void setTarget(Pose2d target) {
        _target = target;
        // You may want to regenerate the trajectory if the target changes significantly.
    }

    public Pose2d getTarget() {
        return _target;
    }

    public CommandSwerveDrivetrain getDrivetrain() {
        return _driveTrain;
    }

    protected static Pose2d transform2dAsPose2d(Transform2d transform) {
        return new Pose2d(transform.getTranslation(), transform.getRotation());
    }

    protected static Transform2d pose2dAsTransform2d(Pose2d pose) {
        return new Transform2d(pose.getTranslation(), pose.getRotation());
    }
}
