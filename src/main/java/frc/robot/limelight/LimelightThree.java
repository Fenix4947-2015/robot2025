package frc.robot.limelight;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.PoseEstimate;

public class LimelightThree extends Limelight {

    private LimelightHelpers.LimelightResults limelightResults;
    private final String identifier;
    private final RobotContainer m_robotContainer;

  public LimelightThree(String identifier, RobotContainer robotContainer) {
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
      return LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(identifier);
    }
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(identifier);
  }

  public double getLatency() {
    LimelightResults results = getLimelightResults();
    return (results.latency_capture + results.latency_jsonParse + results.latency_pipeline) / 1000;
  }

  public void setRotation(double robotYawInDegrees) {
    LimelightHelpers.SetRobotOrientation(identifier, robotYawInDegrees, 0,0,0,0,0);
  }
}

