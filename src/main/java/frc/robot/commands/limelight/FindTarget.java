package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.limelight.Limelight2025;
import frc.robot.limelight.LimelightMegaTagType;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class FindTarget extends Command {
    private final Limelight2025 limelight;
    private final CommandSwerveDrivetrain drivetrain;

    public FindTarget(Limelight2025 limelight, CommandSwerveDrivetrain drivetrain) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        limelight.setActiveFiducuialId(-1);
        drivetrain.setLimelightMegaTagType(LimelightMegaTagType.MEGA_TAG);
        limelight.resetIdFilter();
    }

    @Override
    public void execute() {
        int fiducialId = this.limelight.getFiducialId();
        if (fiducialId > 0) {
            this.limelight.setActiveFiducuialId(fiducialId);
            this.limelight.setIdFilter(fiducialId);
            this.drivetrain.setPoseEstimate(this.limelight.getPoseEstimate());
        }
    }

    @Override
    public boolean isFinished() {
        return this.limelight.getActiveFiducialId() > 0;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setLimelightMegaTagType(LimelightMegaTagType.NONE);
        super.end(interrupted);
    }
}
