package frc.robot.commands.drivetrain;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveSwerveCommand extends Command {

    private CommandSwerveDrivetrain drivetrain;
    private CommandXboxController joystick;
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private final double robotRadius = Math.sqrt(TunerConstants.BackLeft.LocationX * TunerConstants.BackLeft.LocationX +
            TunerConstants.BackLeft.LocationY * TunerConstants.BackLeft.LocationY); // Distance from center of rotation to wheel
    private final double MaxAngularRate = RotationsPerSecond.of(MaxSpeed / robotRadius).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.2) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    public DriveSwerveCommand(CommandSwerveDrivetrain commandSwerveDrivetrain, CommandXboxController joystick) {
        addRequirements(commandSwerveDrivetrain);
        this.drivetrain = commandSwerveDrivetrain;
        this.joystick = joystick;
    }

    @Override
    public void execute() {
        this.drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            );
    }

}
