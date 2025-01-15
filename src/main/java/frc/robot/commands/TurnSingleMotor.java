package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SingleMotor;

public class TurnSingleMotor extends Command{
    private SingleMotor singleMotor;
    private CommandXboxController xboxController;

    public TurnSingleMotor(SingleMotor singleMotor, CommandXboxController xboxController) {
        this.singleMotor = singleMotor;
        addRequirements(this.singleMotor);
        this.xboxController = xboxController;
    }

    @Override
    public void execute() {
        this.singleMotor.run(this.xboxController.getLeftX());
    }
}
