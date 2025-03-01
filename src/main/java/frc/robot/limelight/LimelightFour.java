package frc.robot.limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.PoseEstimate;

public class LimelightFour extends Limelight {

    private LimelightHelpers.LimelightResults limelightResults;
    private final String identifier;
    private final RobotContainer m_robotContainer;

  public LimelightFour(String identifier, RobotContainer robotContainer) {
    super(identifier);
    this.identifier = identifier;
    m_robotContainer = robotContainer;
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

  public double getLatency() {
    LimelightResults results = getLimelightResults();
    return (results.latency_capture + results.latency_jsonParse + results.latency_pipeline) / 1000;
  }

  public double getFiducialId() {
    return LimelightHelpers.getFiducialID(identifier);
  }

  public Pose2d getClosestFiducial() {
    return LimelightHelpers.getLatestResults(identifier).targets_Fiducials[(int) getFiducialId()].getRobotPose_TargetSpace2D();
  }

  public void setRotation(double robotYawInDegrees) {
    LimelightHelpers.SetRobotOrientation(identifier, robotYawInDegrees, 0,0,0,0,0);
  }
}

