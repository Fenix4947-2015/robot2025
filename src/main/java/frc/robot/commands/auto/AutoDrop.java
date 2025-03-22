package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.combo.AutoSequences;
import frc.robot.subsystems.Arm;

public class AutoDrop extends Command {
    private final Arm arm;

    private final Command autoDropCoralL3;
    private final Command autoDropCoralL4;

    private Command currentCommand = null;

    public enum Side {
        LEFT,
        RIGHT
    }

    public AutoDrop(AutoSequences autoSequences, Arm arm, Side side) {
        this.arm = arm;

        autoDropCoralL3 = switch (side) {
            case LEFT -> autoSequences.autoDropCoralL3Left();
            case RIGHT -> autoSequences.autoDropCoralL3Right();
        };
        autoDropCoralL4 = switch (side) {
            case LEFT -> autoSequences.autoDropCoralL4LeftPP();
            case RIGHT -> autoSequences.autoDropCoralL4Right();
        };
    }

    @Override
    public void initialize() {
        if (arm.isArmRetracted()) {
            currentCommand = autoDropCoralL3;
        } else {
            currentCommand = autoDropCoralL4;
        }
        currentCommand.initialize();
    }

    @Override
    public void execute() {
        if (currentCommand != null) {
            currentCommand.execute();
        }
    }

    @Override
    public boolean isFinished() {
        if (currentCommand != null) {
            return currentCommand.isFinished();
        }
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        if (currentCommand != null) {
            currentCommand.end(interrupted);
        }
    }
}
