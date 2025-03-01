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
                return new Pose2d(0,0, Rotation2d.fromDegrees(180));
            } else {
                return new Pose2d(0,0, Rotation2d.fromDegrees(180));
            }
        }
    },
    REEF_3 {
        public Pose2d getPositionForTeam(Alliance team) {
            if (team == Blue) {
                return new Pose2d(0,0, Rotation2d.fromDegrees(-130.5));
            } else {
                return new Pose2d(0,0, Rotation2d.fromDegrees(130.5));
            }
        }
    },
    CORAL_L4_LEFT {
        public Pose2d getPositionForTeam(Alliance team) {
            if (team == Blue) {
                return new Pose2d(1.5, 4, Rotation2d.fromDegrees(135));
            } else {
                return new Pose2d(1.5, 4.2, Rotation2d.fromDegrees(-135));
            }
        }
    },
    CORAL_L4_RIGHT {
        public Pose2d getPositionForTeam(Alliance team) {
            if (team == Blue) {
                return new Pose2d(0.3, 0, Rotation2d.fromDegrees(180));
            } else {
                return new Pose2d(0.3, 0, Rotation2d.fromDegrees(180));
            }
        }
    },
    L4_APPROACH_RIGHT {
        public Pose2d getPositionForTeam(Alliance team) {
            if (team == Blue) {
                return new Pose2d(3.05, 3.82, Rotation2d.fromDegrees(0));
            } else {
                return new Pose2d(3.05, 3.82, Rotation2d.fromDegrees(0));
            }
        }
    },
    STATION_1_APPROACH {
        public Pose2d getPositionForTeam(Alliance team) {
            if (team == Blue) {
                return new Pose2d(1.21, 1.36, Rotation2d.fromDegrees(-122));
            } else {
                return new Pose2d(1.21, 1.36, Rotation2d.fromDegrees(-122));
            }
        }
    },
    STATION_1 {
        public Pose2d getPositionForTeam(Alliance team) {
            if (team == Blue) {
                return new Pose2d(1.06, .87, Rotation2d.fromDegrees(-122));
            } else {
                return new Pose2d(1.06, .87, Rotation2d.fromDegrees(-122));
            }
        }
    },
    NOTE_4 {
        public Pose2d getPositionForTeam(Alliance team) {
            if (team == Blue) {
                return new Pose2d(8.27, 7.457, new Rotation2d());
            } else {
                return new Pose2d(8.27, 0.753, new Rotation2d());
            }
        }
    },
    NOTE_5 {
        public Pose2d getPositionForTeam(Alliance team) {
            if (team == Blue) {
                return new Pose2d(8.27, 5.781, new Rotation2d());
            } else {
                return new Pose2d(8.27, 2.429, new Rotation2d());
            }
        }
    },
    NOTE_6 {
        public Pose2d getPositionForTeam(Alliance team) {
            if (team == Blue) {
                return new Pose2d(8.27, 4.105, new Rotation2d());
            } else {
                return new Pose2d(8.27, 4.105, new Rotation2d());
            }
        }
    },
    NOTE_7 {
        public Pose2d getPositionForTeam(Alliance team) {
            if (team == Blue) {
                return new Pose2d(8.27, 2.429, new Rotation2d());
            } else {
                return new Pose2d(8.27, 5.781
                , new Rotation2d());
            }
        }
    },
    NOTE_8 {
        public Pose2d getPositionForTeam(Alliance team) {
            if (team == Blue) {
                return new Pose2d(8.27, 0.753, new Rotation2d());
            } else {
                return new Pose2d(8.27, 7.457, new Rotation2d());
            }
        }
    },
    NOTE_8_APPROACH {
        public Pose2d getPositionForTeam(Alliance team) {
            if (team == Blue) {
                return new Pose2d(2, 0.753, Rotation2d.fromDegrees(180));
            } else {
                return new Pose2d(2, 7.457, Rotation2d.fromDegrees(180));
            }
        }
    };

    public abstract Pose2d getPositionForTeam(Alliance team);
}
