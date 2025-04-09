package frc.robot.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Fiducial;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.SmartDashboardWrapper;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.CommandSwerveDrivetrain.LimelightToUse;

public class Limelight2025 extends Limelight {

    private LimelightHelpers.LimelightResults limelightResults;
    private final String identifier;
    private final RobotContainer m_robotContainer;
    private int activeFiducuialId;
    private LimelightToUse limelightToUse;

  public Limelight2025(String identifier, LimelightToUse limelightToUse, RobotContainer robotContainer) {
    super(identifier);
    this.identifier = identifier;
    m_robotContainer = robotContainer;
    activeFiducuialId = -1;
    this.limelightToUse = limelightToUse;
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
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(identifier);
  }

  public PoseEstimate getPoseEstimateMegaTag2() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(identifier);
  }

  public double[] getStdevMegaTag() {
    return LimelightHelpers.getStdevMt1(identifier);
  }

  public double[] getStdevMegaTag2() {
    return LimelightHelpers.getStdevMt2(identifier);
  }

  public double getLatency() {
    LimelightResults results = getLimelightResults();
    return (results.latency_capture + results.latency_jsonParse + results.latency_pipeline) / 1000;
  }

  public int getFiducialId() {
    return (int) LimelightHelpers.getFiducialID(identifier);
  }

  public Transform2d getFiducialRobotRelative(LimelightMegaTagType limelightMegaTagType) {
    int fiducialId = getFiducialId();
    PoseEstimate robotPoseEstimate;
    if (limelightMegaTagType == LimelightMegaTagType.MEGA_TAG) {
      robotPoseEstimate = LimelightHelpers.getBotPoseEstimate(identifier);
    } else {
      robotPoseEstimate = LimelightHelpers.getBotPoseEstimate_MegaTag2(identifier);
    }

    Pose2d robotPose = robotPoseEstimate.pose;
    
    if (fiducialId == -1 || robotPose == null || robotPoseEstimate.tagCount < 1) {
      return null;
    }
    Fiducial fiducial = Fiducial.getFiducialById(fiducialId);
    if (fiducial == null) {
      return null;
    }

    return new Transform2d(robotPose, fiducial.getPose2d());
    
  }

  public Fiducial getClosestFiducial() {
    int fiducialId = getFiducialId();
    if (fiducialId == -1) {
      return null;
    }

    return Fiducial.getFiducialById(fiducialId);

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

  public void setIdFilter(int fiducialId) {
    int[] newIds = {fiducialId};
    LimelightHelpers.SetFiducialIDFiltersOverride(identifier, newIds);
  }

  public void resetIdFilter() {
    int[] newIds = {};
    LimelightHelpers.SetFiducialIDFiltersOverride(identifier, newIds);
  }

  public void setRobotOrientation(double headingDeg) {
    LimelightHelpers.SetRobotOrientation(identifier, headingDeg, 0, 0, 0, 0, 0);
  }

  public LimelightToUse getLimelightToUse() {
    return limelightToUse;
  }
}

