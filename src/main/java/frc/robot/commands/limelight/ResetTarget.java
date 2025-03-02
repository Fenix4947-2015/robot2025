package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.limelight.Limelight2025;
import frc.robot.limelight.LimelightMegaTagType;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ResetTarget extends Command {
    private final Limelight2025 limelight;
    private final CommandSwerveDrivetrain drivetrain;

    public ResetTarget(Limelight2025 limelight, CommandSwerveDrivetrain drivetrain) {
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
