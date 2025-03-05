package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * UARMTrajectory generates a trajectory from a start pose to an end pose using a trapezoidal
 * motion profile (uniform acceleration and deceleration) for both translation and rotation.
 * <p>
 * The class is configured with maximum linear speed/acceleration and maximum angular speed/acceleration.
 * It synchronizes the two profiles so that both translation and rotation start and finish at the same time.
 * Call sample(t) to get the State of the trajectory at time t (in seconds).
 */
public class UARMTrajectory {

    // The state returned by the trajectory at a given timestamp.
    public static class State {
        public final double time; // Time in seconds
        public final Pose2d pose;
        public final double linearVelocity;   // in meters per second
        public final double angularVelocity;  // in radians per second
        public final double linearAcceleration;   // in m/s^2
        public final double angularAcceleration;  // in rad/s^2

        public State(double time, Pose2d pose, double linearVelocity, double angularVelocity,
                     double linearAcceleration, double angularAcceleration) {
            this.time = time;
            this.pose = pose;
            this.linearVelocity = linearVelocity;
            this.angularVelocity = angularVelocity;
            this.linearAcceleration = linearAcceleration;
            this.angularAcceleration = angularAcceleration;
        }
    }

    /**
     * A helper class to generate a trapezoidal motion profile for a given distance.
     */
    public static class TrapezoidalProfile {
        public final double distance; // total distance (or angle) to cover
        public final double accel;    // acceleration (or angular acceleration)
        public final double vMax;     // maximum velocity (or angular speed)
        public final double t1;       // time to accelerate
        public final double t2;       // duration of constant (cruise) speed phase
        public final double tTotal;   // total duration of the profile
        public final boolean isTriangular;

        public TrapezoidalProfile(double distance, double accel, double vMax) {
            this.distance = distance;
            this.accel = accel;
            this.vMax = vMax;

            double tAcc = vMax / accel;
            double dAcc = 0.5 * accel * tAcc * tAcc;
            if (2 * dAcc >= distance) {
                // Triangular profile (no constant velocity phase)
                this.isTriangular = true;
                double tPeak = Math.sqrt(distance / accel);
                this.t1 = tPeak;
                this.t2 = 0;
                this.tTotal = 2 * tPeak;
            } else {
                // Trapezoidal profile with a constant velocity phase
                this.isTriangular = false;
                this.t1 = tAcc;
                double dCruise = distance - 2 * dAcc;
                double tCruise = dCruise / vMax;
                this.t2 = tCruise;
                this.tTotal = 2 * tAcc + tCruise;
            }
        }

        public double getPosition(double t) {
            if (t <= 0) return 0;
            if (t >= tTotal) return distance;

            if (isTriangular) {
                double tPeak = t1; // sqrt(distance / accel)
                if (t < tPeak) {
                    return 0.5 * accel * t * t;
                } else {
                    double tDecel = t - tPeak;
                    double dPeak = 0.5 * accel * tPeak * tPeak;
                    double vPeak = accel * tPeak;
                    return dPeak + vPeak * tDecel - 0.5 * accel * tDecel * tDecel;
                }
            } else {
                if (t < t1) {
                    return 0.5 * accel * t * t;
                } else if (t < t1 + t2) {
                    double dAcc = 0.5 * accel * t1 * t1;
                    return dAcc + vMax * (t - t1);
                } else {
                    double tDecel = t - t1 - t2;
                    double dAcc = 0.5 * accel * t1 * t1;
                    double dCruise = vMax * t2;
                    return dAcc + dCruise + vMax * tDecel - 0.5 * accel * tDecel * tDecel;
                }
            }
        }

        public double getVelocity(double t) {
            if (t <= 0 || t >= tTotal) return 0;

            if (isTriangular) {
                double tPeak = t1;
                if (t < tPeak) {
                    return accel * t;
                } else {
                    return accel * (tTotal - t);
                }
            } else {
                if (t < t1) {
                    return accel * t;
                } else if (t < t1 + t2) {
                    return vMax;
                } else {
                    return accel * (tTotal - t);
                }
            }
        }

        public double getAcceleration(double t) {
            if (t < 0 || t > tTotal) return 0;
            if (isTriangular) {
                double tPeak = t1;
                if (t < tPeak) {
                    return accel;
                } else {
                    return -accel;
                }
            } else {
                if (t < t1) {
                    return accel;
                } else if (t < t1 + t2) {
                    return 0;
                } else {
                    return -accel;
                }
            }
        }
    }

    // Trajectory fields
    private final Pose2d startPose;
    private final Pose2d endPose;
    private final TrapezoidalProfile linearProfile;
    private final TrapezoidalProfile angularProfile;
    private final double overallDuration;
    private final double linearTimeScale;
    private final double angularTimeScale;

    /**
     * Constructs a UARMTrajectory.
     *
     * @param startPose       the starting pose
     * @param endPose         the ending pose
     * @param maxLinearSpeed  maximum linear speed (m/s)
     * @param maxLinearAccel  maximum linear acceleration (m/s^2)
     * @param maxAngularSpeed maximum angular speed (rad/s)
     * @param maxAngularAccel maximum angular acceleration (rad/s^2)
     */
    public UARMTrajectory(Pose2d startPose, Pose2d endPose,
                          double maxLinearSpeed, double maxLinearAccel,
                          double maxAngularSpeed, double maxAngularAccel) {
        this.startPose = startPose;
        this.endPose = endPose;

        // Compute translation distance (Euclidean)
        double dx = endPose.getX() - startPose.getX();
        double dy = endPose.getY() - startPose.getY();
        double distance = Math.hypot(dx, dy);
        this.linearProfile = new TrapezoidalProfile(distance, maxLinearAccel, maxLinearSpeed);

        // Compute rotation distance (absolute angular difference in radians)
        double startRot = startPose.getRotation().getRadians();
        double endRot = endPose.getRotation().getRadians();
        double dTheta = endRot - startRot;
        // Normalize to [-pi, pi]
        dTheta = Math.atan2(Math.sin(dTheta), Math.cos(dTheta));
        double absAngle = Math.abs(dTheta);
        this.angularProfile = new TrapezoidalProfile(absAngle, maxAngularAccel, maxAngularSpeed);

        // Determine overall duration as the longer of the two profiles
        double tLinear = linearProfile.tTotal;
        double tAngular = angularProfile.tTotal;
        this.overallDuration = Math.max(tLinear, tAngular);

        // Avoid division by zero when start and end are the same.
        this.linearTimeScale = (tLinear > 0) ? overallDuration / tLinear : 1.0;
        this.angularTimeScale = (tAngular > 0) ? overallDuration / tAngular : 1.0;
    }

    /**
     * Returns the total duration (in seconds) of the trajectory.
     */
    public double getDuration() {
        return overallDuration;
    }

    /**
     * Samples the trajectory at a given time (in seconds) and returns the corresponding State.
     *
     * @param t time in seconds (if t exceeds the duration, the final state is returned)
     * @return the trajectory state at time t
     */
    public State sample(double t) {
        if (t > overallDuration) {
            t = overallDuration;
        }

        // Sample the linear (translation) profile with time scaling
        double tLinear = t / linearTimeScale;
        double linearDistance = linearProfile.getPosition(tLinear);
        double linearVelocity = linearProfile.getVelocity(tLinear) / linearTimeScale;
        double linearAccel = linearProfile.getAcceleration(tLinear) / (linearTimeScale * linearTimeScale);

        // Sample the angular (rotation) profile with time scaling
        double tAngular = t / angularTimeScale;
        double angularDistance = angularProfile.getPosition(tAngular);
        double angularVelocity = angularProfile.getVelocity(tAngular) / angularTimeScale;
        double angularAccel = angularProfile.getAcceleration(tAngular) / (angularTimeScale * angularTimeScale);

        // Interpolate the translation along the straight line from start to end
        double totalDistance = linearProfile.distance;
        double ratio = (totalDistance > 0) ? linearDistance / totalDistance : 1.0;
        double newX = startPose.getX() + (endPose.getX() - startPose.getX()) * ratio;
        double newY = startPose.getY() + (endPose.getY() - startPose.getY()) * ratio;

        // For rotation, use the sign of the total angular difference
        double startRot = startPose.getRotation().getRadians();
        double dTheta = endPose.getRotation().getRadians() - startRot;
        dTheta = Math.atan2(Math.sin(dTheta), Math.cos(dTheta));  // normalize
        double newRot = startRot + Math.signum(dTheta) * angularDistance;

        Pose2d newPose = new Pose2d(newX, newY, new Rotation2d(newRot));
        return new State(t, newPose, linearVelocity, angularVelocity, linearAccel, angularAccel);
    }
}

