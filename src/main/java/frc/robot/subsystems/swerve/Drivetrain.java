// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SmartDashboardWrapper;

import static frc.robot.Constants.ElectricConstants.*;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  
  private static final double SWERVE_TRANSLATION_X = 0.654 / 2; // Front-to-rear
  private static final double SWERVE_TRANSLATION_Y = 0.577 / 2; // SIDE-TO-SIDE
  public static final double K_MAX_SPEED = 15.5 * 1.14 / (12 * 25.4 / 1000); // 3 meters per second
  private static final double K_TURN_RADIUS = Math.sqrt(Math.pow(SWERVE_TRANSLATION_X, 2) + Math.pow(SWERVE_TRANSLATION_Y, 2));
  public static final double K_MAX_ANGULAR_SPEED = K_MAX_SPEED / K_TURN_RADIUS; // 1/2 rotation per second

  public static final double K_MAX_ACCELERATION = 1.0;
  public static final double K_MAX_DECELERATION = 1.0;
  public static final double K_MAX_ANGLUAR_ACCELERATION = K_MAX_ACCELERATION / K_TURN_RADIUS;
  public static final double K_MAX_ANGLUAR_DECCELERATION = K_MAX_DECELERATION / K_TURN_RADIUS;
  
  private final Translation2d m_frontLeftLocation = new Translation2d(SWERVE_TRANSLATION_X, SWERVE_TRANSLATION_Y);
  private final Translation2d m_frontRightLocation = new Translation2d(SWERVE_TRANSLATION_X, -SWERVE_TRANSLATION_Y);
  private final Translation2d m_backLeftLocation = new Translation2d(-SWERVE_TRANSLATION_X, SWERVE_TRANSLATION_Y);
  private final Translation2d m_backRightLocation = new Translation2d(-SWERVE_TRANSLATION_X, -SWERVE_TRANSLATION_Y);

  public record SwerveModuleSettings(int id, int driveMotorChannel, int turningMotorChannel, int turningEncoderId) {}

  private final SwerveModule m_frontLeft = new SwerveModule(kSwerveModuleSettingsFL, 0, false);
  private final SwerveModule m_frontRight = new SwerveModule(kSwerveModuleSettingsFR, 0, false);
  private final SwerveModule m_backLeft = new SwerveModule(kSwerveModuleSettingsBL, 0, false);
  private final SwerveModule m_backRight = new SwerveModule(kSwerveModuleSettingsBR, 0, false);

  private double speedRatio;
  private final Pigeon2 m_gyro = new Pigeon2(kPigeon2Channel, "rio");

  private SwerveDriveKinematics m_kinematics;

  private SwerveDriveOdometry m_odometry;

  private double gyroOffset;

  private final PIDController m_rotationPIDController = new PIDController(1.0, 0.0, 0.0);

  private double m_xSpeed;
  private double m_ySpeed;
  private double m_rot;
  boolean m_fieldRelative;

  public Drivetrain(double speedRatio) {
    resetGyro(new Pose2d(0,0, Rotation2d.fromDegrees(0)));
    this.speedRatio = speedRatio;
    this.gyroOffset = 0;
    m_rotationPIDController.setSetpoint(0.0);
    configureAutoBuilder();
  }
  
  private void configureAutoBuilder() {
    try {
        var config = RobotConfig.fromGUISettings();
        System.out.println("Configure AutoBuilder");
          AutoBuilder.configure(
                () -> getOdometry(),   // Supplier of current robot pose
                this::resetOdometry,         // Consumer for seeding pose against auto
                () -> getVelocity(), // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,speeds.omegaRadiansPerSecond,false),/*  setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),*/
                new PPHolonomicDriveController(
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void driveNormalized(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    //System.out.println(Instant.now() + " " + getClass().getSimpleName() + ".driveNormalized()");

    double robotXSpeed = xSpeed * K_MAX_SPEED * speedRatio;
    double robotYSpeed = ySpeed * K_MAX_SPEED * speedRatio;
    double robotRotSpeed = rot * K_MAX_ANGULAR_SPEED * speedRatio;
    drive(robotXSpeed, robotYSpeed, robotRotSpeed, fieldRelative);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    //System.out.println(Instant.now() + " " + getClass().getSimpleName() + ".drive()");

    m_xSpeed = xSpeed;
    m_ySpeed = ySpeed;
    m_rot = rot;
    m_fieldRelative = fieldRelative;
  }

  public void driveRelative(double xSpeed, double ySpeed) {
    //System.out.println(Instant.now() + " " + getClass().getSimpleName() + ".drive()");

    m_xSpeed = xSpeed;
    m_ySpeed = ySpeed;
  }

  @Override
  public void periodic() {
    //System.out.println(Instant.now() + " " + getClass().getSimpleName() + ".periodic()");

    SwerveModuleState[] swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            m_fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(m_xSpeed, m_ySpeed, m_rot, getAdjustedRotation2D())
                : new ChassisSpeeds(m_xSpeed, m_ySpeed, m_rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, K_MAX_SPEED * speedRatio);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
    updateOdometry();
    getOdometry();
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        getAdjustedRotation2D(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }

  public SwerveModule getSwerveModuleFrontRight() {
    return m_frontRight;
  }

  public SwerveModule getSwerveModuleBackRight() {
    return m_backRight;
  }
    
  public SwerveModule getSwerveModuleFrontLeft() {
    return m_frontLeft;
  }
    
  public SwerveModule getSwerveModuleBackLeft() {
    return m_backLeft;
  }

  public double getGyroAngle() {
    return getAdjustedRotation2D().getRadians();
  }

  public Pose2d getOdometry() {
    Pose2d currentPose = m_odometry.getPoseMeters();

    SmartDashboardWrapper.putNumber("currentPoseX", currentPose.getX());
    SmartDashboardWrapper.putNumber("currentPoseY", currentPose.getY());
    SmartDashboardWrapper.putNumber("currentPoseAngle", currentPose.getRotation().getDegrees());

    return currentPose;
  }

  public ChassisSpeeds getVelocity() {

    ChassisSpeeds chassisSpeeds = m_kinematics.toChassisSpeeds(
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_backLeft.getState(),
      m_backRight.getState()
      );

      return chassisSpeeds;
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(getAdjustedRotation2D(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        },
        pose);
  }

  public void resetGyro(Pose2d pose) {
    m_gyro.reset();
    gyroOffset = pose.getRotation().getDegrees();
    m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    m_odometry = new SwerveDriveOdometry(
      m_kinematics,
      getAdjustedRotation2D(),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition()
      }, 
      pose);
    resetOdometry(pose);
  }

  public boolean isMovingSlow() {
    ChassisSpeeds speeds = getVelocity();
    if (Math.abs(speeds.omegaRadiansPerSecond) < 0.01 && 
    Math.abs(speeds.vxMetersPerSecond) < 0.1 && 
    Math.abs(speeds.vyMetersPerSecond) < 0.1) {
      return true;
    }
    return false;
  }

  public void updateSmartDashboard() {
    SmartDashboardWrapper.putNumber("wheelAngleFrontRight", getSwerveModuleFrontRight().getWheelAngle());
    SmartDashboardWrapper.putNumber("wheelAngleBackRight", getSwerveModuleBackRight().getWheelAngle());
    SmartDashboardWrapper.putNumber("wheelAngleFrontLeft", getSwerveModuleFrontLeft().getWheelAngle());
    SmartDashboardWrapper.putNumber("wheelAngleBackLeft", getSwerveModuleBackLeft().getWheelAngle());

    SmartDashboardWrapper.putNumber("stateAngleFrontRight", getSwerveModuleFrontRight().getStateAngle());
    SmartDashboardWrapper.putNumber("stateAngleBackRight", getSwerveModuleBackRight().getStateAngle());
    SmartDashboardWrapper.putNumber("stateAngleFrontLeft", getSwerveModuleFrontLeft().getStateAngle());
    SmartDashboardWrapper.putNumber("stateAngleBackLeft", getSwerveModuleBackLeft().getStateAngle());

    SmartDashboardWrapper.putNumber("gyroAngle", getGyroAngle());
    SmartDashboardWrapper.putNumber("adjustedRotation2d", getAdjustedRotation2D().getDegrees());
    SmartDashboardWrapper.putNumber("gyroOffset", gyroOffset);
  }

  public Rotation2d getAdjustedRotation2D() {
        // voir https://store.ctr-electronics.com/content/user-manual/Pigeon2%20User's%20Guide.pdf
    // getYaw() est l'équivalent de getFusedHeading() du pigeon 1
    return Rotation2d.fromDegrees(m_gyro.getRotation2d().getDegrees() + gyroOffset);
  }
}
