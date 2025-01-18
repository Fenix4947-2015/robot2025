// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.drivetrain.DriveSwerve;
import frc.robot.subsystems.swerve.Drivetrain;

public class RobotContainer {
  private final double SPEED_RATIO = 1.0;

  public final SmartDashboardSettings m_smartDashboardSettings = new SmartDashboardSettings();

  public final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  public final Drivetrain m_driveTrain = new Drivetrain(SPEED_RATIO);

  private final DriveSwerve m_driveSwerve = new DriveSwerve(m_driverController, m_driveTrain, SPEED_RATIO);

  public RobotContainer() {
    configureBindings();
    configureDefaultCommands();
  }

  private void configureBindings() {
  }

  private void configureDefaultCommands() {
    m_driveTrain.setDefaultCommand(m_driveSwerve);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
