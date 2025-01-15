// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.RunSingleMotor;
import frc.robot.commands.TurnSingleMotor;
import frc.robot.subsystems.SingleMotor;

public class RobotContainer {

  public final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  public final SingleMotor motor11 = new SingleMotor(11);
  public final SingleMotor motor21 = new SingleMotor(21);
  public final SingleMotor motor31 = new SingleMotor(31);
  public final SingleMotor motor41 = new SingleMotor(41);
  public final SingleMotor motor12 = new SingleMotor(12);
  public final SingleMotor motor22 = new SingleMotor(22);
  public final SingleMotor motor32 = new SingleMotor(32);
  public final SingleMotor motor42 = new SingleMotor(42);

  public final RunSingleMotor runMotor11 = new RunSingleMotor(motor11, m_driverController);
  public final RunSingleMotor runMotor21 = new RunSingleMotor(motor21, m_driverController);
  public final RunSingleMotor runMotor31 = new RunSingleMotor(motor31, m_driverController);
  public final RunSingleMotor runMotor41 = new RunSingleMotor(motor41, m_driverController);
  public final TurnSingleMotor turnMotor12 = new TurnSingleMotor(motor12, m_driverController);
  public final TurnSingleMotor turnMotor22 = new TurnSingleMotor(motor22, m_driverController);
  public final TurnSingleMotor turnMotor32 = new TurnSingleMotor(motor32, m_driverController);
  public final TurnSingleMotor turnMotor42 = new TurnSingleMotor(motor42, m_driverController);

  public RobotContainer() {
    configureBindings();
    configureDefaultCommands();
  }

  private void configureBindings() {
  }

  private void configureDefaultCommands() {
    motor11.setDefaultCommand(runMotor11);
    motor21.setDefaultCommand(runMotor21);
    motor31.setDefaultCommand(runMotor31);
    motor41.setDefaultCommand(runMotor41);
    motor12.setDefaultCommand(turnMotor12);
    motor22.setDefaultCommand(turnMotor22);
    motor32.setDefaultCommand(turnMotor32);
    motor42.setDefaultCommand(turnMotor42);
}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
