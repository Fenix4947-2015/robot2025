package frc.robot.commands.drivetrain;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.generated.TunerConstants;
import frc.robot.limelight.LimelightThree;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveSwerveCommand extends Command {

    private static final boolean APPLY_SLEW_RATE_LIMITER = false;
    private static final boolean APPLY_DEADBAND = true;

    private final SlewRateLimiter filterVelocityX = new SlewRateLimiter(1.0);
    private final SlewRateLimiter filterVelocityY = new SlewRateLimiter(1.0);
    private final SlewRateLimiter filterRotRate = new SlewRateLimiter(1.0);

    private CommandSwerveDrivetrain drivetrain;
    private CommandXboxController joystick;
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private final double RobotRadius = Math.sqrt(TunerConstants.BackLeft.LocationX * TunerConstants.BackLeft.LocationX +
            TunerConstants.BackLeft.LocationY * TunerConstants.BackLeft.LocationY); // Distance from center of rotation to wheel
    private final double MaxAngularRate = RadiansPerSecond.of(MaxSpeed / RobotRadius).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            //.withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.2) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final LimelightThree limelightThree;

    public DriveSwerveCommand(CommandSwerveDrivetrain commandSwerveDrivetrain, CommandXboxController joystick, LimelightThree limelightThree) {
        addRequirements(commandSwerveDrivetrain);
        this.drivetrain = commandSwerveDrivetrain;
        this.joystick = joystick;
        this.limelightThree = limelightThree;
    }

    @Override
    public void execute() {

        double leftYInv = APPLY_SLEW_RATE_LIMITER ? filterVelocityY.calculate(-joystick.getLeftY()) : -joystick.getLeftY();
        double leftXInv = APPLY_SLEW_RATE_LIMITER ? filterVelocityX.calculate(-joystick.getLeftX()) : -joystick.getLeftX();
        double rightXInv = APPLY_SLEW_RATE_LIMITER ? filterRotRate.calculate(-joystick.getRightX()) : -joystick.getRightX();

        double leftYInvDb = APPLY_DEADBAND ? MathUtil.applyDeadband(leftYInv, 0.2) : leftYInv;
        double leftXInvDb = APPLY_DEADBAND ? MathUtil.applyDeadband(leftXInv, 0.2) : leftXInv;
        double rightXInvDb = APPLY_DEADBAND ? MathUtil.applyDeadband(rightXInv, 0.2) : rightXInv;

        double leftYInvDbSq = Math.pow(leftYInvDb, 2.0) * Math.signum(leftYInvDb);
        double leftXInvDbSq = Math.pow(leftXInvDb, 2.0) * Math.signum(leftXInvDb);
        double rightInvDbSq = Math.pow(rightXInvDb, 2.0) * Math.signum(rightXInvDb);

        var driveState = drivetrain.getState();
        double headingDeg = drivetrain.getPigeon2().getYaw().getValueAsDouble();
        double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

        LimelightHelpers.SetRobotOrientation("limelight", headingDeg, 0, 0, 0, 0, 0);
        PoseEstimate llMeasurement = limelightThree.getPoseEstimate();
        if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
                drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
        }

        this.drivetrain.applyRequest(() ->
                drive.withVelocityX(leftYInvDbSq * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(leftXInvDbSq * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(rightInvDbSq * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
            .execute();
    }

}
