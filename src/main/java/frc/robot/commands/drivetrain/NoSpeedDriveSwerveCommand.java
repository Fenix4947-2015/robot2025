package frc.robot.commands.drivetrain;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.limelight.Limelight2025;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class NoSpeedDriveSwerveCommand extends Command {

    private CommandSwerveDrivetrain drivetrain;
    private Limelight2025 limelight;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

    public NoSpeedDriveSwerveCommand(CommandSwerveDrivetrain commandSwerveDrivetrain, Limelight2025 limelight) {
        addRequirements(commandSwerveDrivetrain);
        this.drivetrain = commandSwerveDrivetrain;
        this.limelight = limelight;
    }
    
    @Override
    public void initialize() {
        if (limelight.getActiveFiducialId() > 0) {
            return;
        };

        this.drivetrain.applyRequest(() ->
                drive.withVelocityX(0) 
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .execute();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
