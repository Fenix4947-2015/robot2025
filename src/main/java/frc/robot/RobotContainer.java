// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.arm.MoveArmDirect;
import frc.robot.commands.arm.MoveArmDropPosition;
import frc.robot.commands.auto.AutoDrop;
import frc.robot.commands.auto.DrivetrainPathFollower;
import frc.robot.commands.balls.RollBalls;
import frc.robot.commands.combo.AutoSequences;
import frc.robot.commands.coralgripper.WaitForCoral;
import frc.robot.commands.drivetrain.DriveSwerveCommand;
import frc.robot.commands.winch.RollCageGripper;
import frc.robot.commands.winch.RollWinchSpeed;
import frc.robot.commands.winch.RollWinchStick;
import frc.robot.generated.TunerConstants;
import frc.robot.limelight.Limelight2025;
import frc.robot.subsystems.*;
import frc.robot.subsystems.CommandSwerveDrivetrain.LimelightToUse;
import frc.robot.commands.WaitSmartDashBoard;

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

    public final Limelight2025 limelightThree = new Limelight2025("limelight-three", LimelightToUse.LIMELIGHT_THREE,this);
    public final Limelight2025 limelightFour = new Limelight2025("limelight-four", LimelightToUse.LIMELIGHT_FOUR, this);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain(limelightFour, limelightThree);
    public final CoralGripper m_coralGripper = new CoralGripper();
    public final Arm m_arm = new Arm(m_coralGripper);
    public final Balls m_balls = new Balls();
    public final Winch m_winch = new Winch();
    public final CageGripper m_cageGripper = new CageGripper();
    public final SmartDashboardSettings smartDashboardSettings = new SmartDashboardSettings();

    private final DriveSwerveCommand driveSwerveCommand = new DriveSwerveCommand(drivetrain, m_driverController, limelightFour);
    private final MoveArmDirect m_moveArmDirect = new MoveArmDirect(m_arm, m_helperController,m_driverController);
    private final MoveArmDropPosition m_moveArmL4 = new MoveArmDropPosition(m_arm, Arm.DropPosition.L4);
    private final MoveArmDropPosition m_moveArmL3 = new MoveArmDropPosition(m_arm, Arm.DropPosition.L3);
    private final MoveArmDropPosition m_moveArmL2 = new MoveArmDropPosition(m_arm, Arm.DropPosition.L2);
    private final MoveArmDropPosition m_moveArmLow = new MoveArmDropPosition(m_arm, Arm.DropPosition.LOW);
    private final RollBalls m_rollBalls = new RollBalls(m_balls,m_helperController);
    private final RollWinchStick m_rollWinchStick = new RollWinchStick(m_winch, m_helperController);
    private final RollWinchSpeed m_stopWinch = new RollWinchSpeed(m_winch, 0.0);
    private final RollCageGripper m_rollCageGripper = new RollCageGripper(m_cageGripper);
    private final RollCageGripper m_stopCageGripper = new RollCageGripper(m_cageGripper, 0.0);
    private final Command autoDropCoralRight = new AutoDrop(new AutoSequences(this), m_arm, AutoDrop.Side.RIGHT);
    private final Command autoDropCoralLeft = new AutoDrop(new AutoSequences(this), m_arm, AutoDrop.Side.LEFT);
    private final Command autoPickupCoralStation1 = new AutoSequences(this).autoPickupCoralStation1PP();
    private final Command gripCoral = new AutoSequences(this).gripCoral();
    private final Command autoPathPlanner = new AutoSequences(this).autoMoveCoralL4LeftPPTest();

    // Combo commands
    private final Command m_clampCoral = m_autoSequences.clampCoral();

    public Alliance m_alliance = Alliance.Red;
    /* Path follower */
    private final SendableChooser<Command> autoChooser;
    public double auto_delay = 0.0;

    public RobotContainer() {
        autoChooser = buildAutoChooser();
        configureBindings();
        configureDefaultCommands();
    }

    private SendableChooser<Command> buildAutoChooser() {
        final SendableChooser<Command> autoChooser;
        NamedCommands.registerCommand("toggle side gripper",new InstantCommand(m_coralGripper::toggleSideGripper, m_coralGripper));
        NamedCommands.registerCommand("auto dunk coral right", autoDropCoralRight);
        NamedCommands.registerCommand("auto dunk coral left", autoDropCoralLeft);
        NamedCommands.registerCommand("auto get coral station 1",autoPickupCoralStation1);
        NamedCommands.registerCommand("Arm L4",m_moveArmL4);
        NamedCommands.registerCommand("Arm Lowest",m_moveArmLow);
        NamedCommands.registerCommand("Grip Coral",gripCoral);
        NamedCommands.registerCommand("Auto Delay",new WaitSmartDashBoard(smartDashboardSettings));
        autoChooser = AutoBuilder.buildAutoChooser("auto_path");
        SmartDashboard.putData("Auto Mode", autoChooser);
        SmartDashboard.putNumber("Auto Delay", auto_delay);
        return autoChooser;
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        //m_driverController.b().whileTrue(drivetrain.applyRequest(() ->
        //        point.withModuleDirection(new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))
        //));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        m_driverController.back().and(m_driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        m_driverController.back().and(m_driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        m_driverController.start().and(m_driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        m_driverController.start().and(m_driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        m_driverController.start().and(m_driverController.back()).onTrue(new InstantCommand(logger::stop));

        /*
        m_driverController.rightBumper().whileTrue(autoDropCoralRight);
        m_driverController.leftBumper().whileTrue(autoDropCoralLeft);
        m_driverController.povLeft().whileTrue(autoPathPlanner);
        m_driverController.y().whileTrue(autoPickupCoralStation1);
        m_driverController.a().whileTrue(m_moveArmLow);
        m_driverController.x().whileTrue(m_moveArmL3);
        m_driverController.b().whileTrue(m_moveArmL4);

        // reset the field-centric heading on left bumper press
        m_driverController.back().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        //m_driverController.rightTrigger().onChange(m_moveArmDirect);
        //m_driverController.leftTrigger().onChange(m_moveArmDirect);

        drivetrain.registerTelemetry(logger::telemeterize);

        //m_helperController.leftStick().onTrue(m_moveArmDirect);
        m_helperController.rightStick().whileTrue(m_rollWinchStick);

        m_helperController.povLeft().onTrue(m_autoSequences.waitAndClampCoral());
        m_helperController.povRight().onTrue(m_autoSequences.freeCoral());
        m_helperController.povDown().onTrue(m_autoSequences.dropCoral());
        m_helperController.povUp().onTrue(new InstantCommand(m_coralGripper::openSideGripper, m_arm));
        m_helperController.x().onTrue(new InstantCommand(m_arm::toggleExtender, m_arm));
        m_helperController.start().whileTrue(m_rollCageGripper);
        m_helperController.leftBumper().whileTrue(m_moveArmL3);
        m_helperController.rightBumper().whileTrue(m_moveArmL4);
        m_helperController.back().whileTrue(m_moveArmLow);
        m_helperController.a().onTrue(new InstantCommand(m_coralGripper::toggleFrontGripper, m_coralGripper));
        m_helperController.b().onTrue(new InstantCommand(m_coralGripper::toggleSideGripper, m_coralGripper));
        m_helperController.y().onTrue(new InstantCommand(m_coralGripper::togglePusher, m_coralGripper));
        */
    }

    public void configureDefaultCommands() {
        drivetrain.setDefaultCommand(driveSwerveCommand);
        m_arm.setDefaultCommand(m_moveArmDirect);
        m_winch.setDefaultCommand(m_stopWinch);
        m_balls.setDefaultCommand(m_rollBalls);
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

    public void robotPeriodic() {
        SmartDashboardWrapper.putNumber("Match Time", DriverStation.getMatchTime());
    }
}
