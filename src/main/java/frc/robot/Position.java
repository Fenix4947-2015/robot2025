package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;

public enum Position {
    REEF_1 {
        public Pose2d getPositionForTeam(Alliance team) {
            if (team == Blue) {
                return new Pose2d(0, 0, Rotation2d.fromDegrees(180));
            } else {
                return new Pose2d(0, 0, Rotation2d.fromDegrees(180));
            }
        }
    },
    REEF_2 {
        public Pose2d getPositionForTeam(Alliance team) {
            if (team == Blue) {
                return new Pose2d(0, 0, Rotation2d.fromDegrees(180));
            } else {
                return new Pose2d(0, 0, Rotation2d.fromDegrees(180));
            }
        }
    },
    REEF_3 {
        public Pose2d getPositionForTeam(Alliance team) {
            if (team == Blue) {
                return new Pose2d(0, 0, Rotation2d.fromDegrees(-130.5));
            } else {
                return new Pose2d(0, 0, Rotation2d.fromDegrees(130.5));
            }
        }
    },
    CORAL_L3_RIGHT {
        public Pose2d getPositionForTeam(Alliance team) {
            return new Pose2d(0.52, 0.23, Rotation2d.fromDegrees(180));
        }
    },
    L3_APPROACH_RIGHT {
        public Pose2d getPositionForTeam(Alliance team) {
            return new Pose2d(1.2, 0.23, Rotation2d.fromDegrees(180));
        }
    },
    CORAL_L4_RIGHT {
        public Pose2d getPositionForTeam(Alliance team) {
            return new Pose2d(0.48, 0.23, Rotation2d.fromDegrees(180));
        }
    },
    L4_APPROACH_RIGHT {
        public Pose2d getPositionForTeam(Alliance team) {
            return new Pose2d(1, 0.23, Rotation2d.fromDegrees(180));
        }
    },
    CORAL_L3_LEFT {
        public Pose2d getPositionForTeam(Alliance team) {
            return new Pose2d(0.52, -0.11, Rotation2d.fromDegrees(180));
        }
    },
    L3_APPROACH_LEFT {
        public Pose2d getPositionForTeam(Alliance team) {
            return new Pose2d(1.2, -0.11, Rotation2d.fromDegrees(180));
        }
    },
    CORAL_L4_LEFT {
        public Pose2d getPositionForTeam(Alliance team) {
            return new Pose2d(0.48, -0.11, Rotation2d.fromDegrees(180));
        }
    },
    L4_APPROACH_LEFT {
        public Pose2d getPositionForTeam(Alliance team) {
            return new Pose2d(1, -0.11, Rotation2d.fromDegrees(180));
        }
    },
    STATION_1_APPROACH {
        public Pose2d getPositionForTeam(Alliance team) {
            return new Pose2d(1, 0, Rotation2d.fromDegrees(180));
        }
    },
    STATION_1 {
        public Pose2d getPositionForTeam(Alliance team) {
            return new Pose2d(0.48, 0, Rotation2d.fromDegrees(180));
        }
    };

    public abstract Pose2d getPositionForTeam(Alliance team);
}
