package frc.robot.commands.drivetrain;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class NoSpeedDriveSwerveCommand extends Command {

    private CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); 

    public NoSpeedDriveSwerveCommand(CommandSwerveDrivetrain commandSwerveDrivetrain) {
        addRequirements(commandSwerveDrivetrain);
        this.drivetrain = commandSwerveDrivetrain;
    }
    
    @Override
    public void execute() {

        this.drivetrain.applyRequest(() ->
                drive.withVelocityX(0) 
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .execute();
    }

}
