// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.arm.KeepArmInPosition;
import frc.robot.commands.arm.MoveArmDirect;
import frc.robot.commands.arm.MoveArmDirectDriverDown;
import frc.robot.commands.arm.MoveArmDirectDriverUp;
import frc.robot.commands.arm.MoveArmDropPosition;
import frc.robot.commands.arm.StopArm;
import frc.robot.commands.balls.RollBalls;
import frc.robot.commands.balls.RollBallsClockWise;
import frc.robot.commands.combo.AutoSequences;
import frc.robot.commands.drivetrain.DriveSwerveCommand;
import frc.robot.commands.winch.RollCageGripper;
import frc.robot.commands.winch.RollWinchSpeed;
import frc.robot.commands.winch.RollWinchStick;
import frc.robot.generated.TunerConstants;
import frc.robot.limelight.LimelightFour;
import frc.robot.subsystems.*;

import static edu.wpi.first.units.Units.MetersPerSecond;

public class RobotContainer {
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    private final CommandXboxController m_helperController = new CommandXboxController(OperatorConstants.kHelperControllerPort);
    
    private final AutoSequences m_autoSequences = new AutoSequences(this);

    // Subsystems

    public final LimelightFour limelightFour = new LimelightFour("limelight", this);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(limelightFour);
    public final CoralGripper m_coralGripper = new CoralGripper();
    public final Arm m_arm = new Arm(m_coralGripper);
    public final Balls m_balls = new Balls();
    public final Winch m_winch = new Winch();
    public final CageGripper m_cageGripper = new CageGripper();

    private final DriveSwerveCommand driveSwerveCommand = new DriveSwerveCommand(drivetrain, m_driverController, limelightFour);
    private final StopArm m_stopArm = new StopArm(m_arm);
    private final KeepArmInPosition m_keepArmInPosition = new KeepArmInPosition(m_arm);
    private final MoveArmDirect m_moveArmDirect = new MoveArmDirect(m_arm, m_helperController, m_keepArmInPosition);
    private final MoveArmDirectDriverUp m_moveArmDriverUp = new MoveArmDirectDriverUp(m_arm, m_keepArmInPosition);
    private final MoveArmDirectDriverDown m_moveArmDriverDown = new MoveArmDirectDriverDown(m_arm, m_keepArmInPosition);
    private final MoveArmDropPosition m_moveArmL4 = new MoveArmDropPosition(m_arm, Arm.DropPosition.L4);
    private final MoveArmDropPosition m_moveArmL3 = new MoveArmDropPosition(m_arm, Arm.DropPosition.L3);
    private final MoveArmDropPosition m_moveArmL2 = new MoveArmDropPosition(m_arm, Arm.DropPosition.L2);
    private final RollBalls m_rollBalls = new RollBalls(m_balls);
    private final RollBallsClockWise m_rollBallsClockWise = new RollBallsClockWise(m_balls);
    private final RollWinchStick m_rollWinchStick = new RollWinchStick(m_winch, m_helperController);
    private final RollWinchSpeed m_stopWinch = new RollWinchSpeed(m_winch, 0.0);
    private final RollCageGripper m_rollCageGripper = new RollCageGripper(m_cageGripper);
    private final RollCageGripper m_stopCageGripper = new RollCageGripper(m_cageGripper, 0.0);

    // Combo commands
    private final Command m_clampCoral = m_autoSequences.clampCoral();

    public Alliance m_alliance = Alliance.Red;
    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        NamedCommands.registerCommand("toggle side gripper",new InstantCommand(m_coralGripper::toggleSideGripper, m_coralGripper));
        autoChooser = AutoBuilder.buildAutoChooser("auto_path");
        SmartDashboard.putData("Auto Mode", autoChooser);


        configureBindings();
        configureDefaultCommands();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        m_driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        m_driverController.b().whileTrue(drivetrain.applyRequest(() ->
                point.withModuleDirection(new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        //m_driverController.back().and(m_driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        //m_driverController.back().and(m_driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        //m_driverController.start().and(m_driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        //m_driverController.start().and(m_driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        //m_driverController.rightBumper().onTrue(new InstantCommand(logger::stop));

        // reset the field-centric heading on left bumper press
        m_driverController.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        m_driverController.rightTrigger().whileTrue(m_moveArmDriverUp);
        m_driverController.leftTrigger().whileTrue(m_moveArmDriverDown);

        drivetrain.registerTelemetry(logger::telemeterize);

        m_helperController.leftStick().whileTrue(m_moveArmDirect);
        m_helperController.rightStick().onTrue(m_rollWinchStick);

        m_helperController.povLeft().onTrue(m_autoSequences.clampCoral());
        m_helperController.povRight().onTrue(m_autoSequences.freeCoral());
        m_helperController.povDown().onTrue(m_autoSequences.dropCoral());
        m_helperController.povUp().onTrue(new InstantCommand(m_coralGripper::openSideGripper, m_arm));
        m_helperController.x().onTrue(new InstantCommand(m_arm::toggleExtender, m_arm));
        m_helperController.start().whileTrue(m_rollCageGripper);
        m_helperController.y().whileTrue(m_moveArmL2);
        m_helperController.leftBumper().whileTrue(m_moveArmL3);
        m_helperController.rightBumper().whileTrue(m_moveArmL4);
        m_helperController.a().onTrue(new InstantCommand(m_coralGripper::toggleFrontGripper, m_coralGripper));
        m_helperController.b().onTrue(new InstantCommand(m_coralGripper::toggleSideGripper, m_coralGripper));
        m_helperController.rightTrigger().whileTrue(m_rollBalls);
        m_helperController.leftTrigger().whileTrue(m_rollBallsClockWise);

    }

    public void configureDefaultCommands() {
        drivetrain.setDefaultCommand(driveSwerveCommand);
        m_arm.setDefaultCommand(m_keepArmInPosition);
        m_balls.setDefaultCommand(m_rollBalls);
        m_winch.setDefaultCommand(m_stopWinch);
        m_cageGripper.setDefaultCommand(m_stopCageGripper);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void setAlliance() {
        m_alliance = DriverStation.getAlliance().orElse(m_alliance);
    }

    public void teleopInit() {
        setAlliance();
    }

    public void autonomousInit() {
        setAlliance();
    }
}
