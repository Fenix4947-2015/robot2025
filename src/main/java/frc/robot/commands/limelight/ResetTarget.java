package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.limelight.LimelightFour;
import frc.robot.limelight.LimelightMegaTagType;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ResetTarget extends Command {
    private final LimelightFour limelight;
    private final CommandSwerveDrivetrain drivetrain;

    public ResetTarget(LimelightFour limelight, CommandSwerveDrivetrain drivetrain) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        limelight.setActiveFiducuialId(-1);
        drivetrain.setLimelightMegaTagType(LimelightMegaTagType.NONE);
        limelight.resetIdFilter();
    }
}
