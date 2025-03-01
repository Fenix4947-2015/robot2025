package frc.robot.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Fiducial;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.LimelightHelpers.PoseEstimate;

public class LimelightFour extends Limelight {

    private LimelightHelpers.LimelightResults limelightResults;
    private final String identifier;
    private final RobotContainer m_robotContainer;
    private int activeFiducuialId;

  public LimelightFour(String identifier, RobotContainer robotContainer) {
    super(identifier);
    this.identifier = identifier;
    m_robotContainer = robotContainer;
    activeFiducuialId = -1;
  }

  @Override
  public void periodic() {
    super.periodic();
    limelightResults = LimelightHelpers.getLatestResults(identifier);
  }

  public LimelightHelpers.LimelightResults getLimelightResults() {
    return limelightResults;
  }
  
  public PoseEstimate getPoseEstimate() {
    if (DriverStation.Alliance.Red.equals(m_robotContainer.m_alliance)) {
      return LimelightHelpers.getBotPoseEstimate_wpiRed(identifier);
    }
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(identifier);
  }

  public PoseEstimate getPoseEstimateMegaTag2() {
    if (DriverStation.Alliance.Red.equals(m_robotContainer.m_alliance)) {
      return LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(identifier);
    }
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(identifier);
  }

  public double getLatency() {
    LimelightResults results = getLimelightResults();
    return (results.latency_capture + results.latency_jsonParse + results.latency_pipeline) / 1000;
  }

  public int getFiducialId() {
    return (int) LimelightHelpers.getFiducialID(identifier);
  }

  public Transform2d getClosestFiducial() {
    int fiducialId = getFiducialId();
    Pose2d robotPose = LimelightHelpers.getBotPose2d(identifier);
    if (fiducialId == -1 || robotPose == null) {
      return null;
    }
    Fiducial fiducial = Fiducial.getFiducialById(fiducialId);
    if (fiducial == null) {
      return null;
    }
    return new Transform2d(robotPose, fiducial.getPose2d());
  }

  public void setRotation(double robotYawInDegrees) {
    LimelightHelpers.SetRobotOrientation(identifier, robotYawInDegrees, 0,0,0,0,0);
  }

  public void setActiveFiducuialId(int fiducialId) {
    this.activeFiducuialId = fiducialId;
  }

  public int getActiveFiducialId() {
    return activeFiducuialId;
  }
}

