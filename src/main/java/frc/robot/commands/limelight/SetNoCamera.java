package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.limelight.Limelight2025;
import frc.robot.limelight.LimelightMegaTagType;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class SetNoCamera extends Command {
    private final Limelight2025 limelight;
    private final CommandSwerveDrivetrain drivetrain;

    public SetNoCamera(Limelight2025 limelight, CommandSwerveDrivetrain drivetrain) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        drivetrain.setLimelightMegaTagType(LimelightMegaTagType.NONE);
        drivetrain.setLimelightToUse(limelight.getLimelightToUse());
        limelight.resetIdFilter();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
