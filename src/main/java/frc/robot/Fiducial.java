package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public enum Fiducial {
    // Enum constants corresponding to each fiducial in the JSON.
    FIDUCIAL_1("apriltag3_36h11_classic", 1, 165.1, 
        new double[]{-0.5877852522924729, -0.8090169943749473, 0, 7.923198000000001,
                     0.8090169943749473, -0.5877852522924729, 0, -3.3706799999999997,
                     0, 0, 1, 1.4859, 0, 0, 0, 1}, true),
                     
    FIDUCIAL_2("apriltag3_36h11_classic", 2, 165.1, 
        new double[]{-0.5877852522924734, 0.8090169943749473, 0, 7.923198000000001,
                     -0.8090169943749473, -0.5877852522924734, 0, 3.3704799999999997,
                     0, 0, 1, 1.4859, 0, 0, 0, 1}, true),
                     
    FIDUCIAL_3("apriltag3_36h11_classic", 3, 165.1, 
        new double[]{-2.220446049250313e-16, 1, 0, 2.786809999999999,
                     -1, -2.220446049250313e-16, 0, 4.02961,
                     0, 0, 1, 1.30175, 0, 0, 0, 1}, true),
                     
    FIDUCIAL_4("apriltag3_36h11_classic", 4, 165.1, 
        new double[]{0.8660254037844387, 0, 0.49999999999999994, 0.5020799999999994,
                     0, 1, 0, 2.111656,
                     -0.49999999999999994, 0, 0.8660254037844387, 1.8679160000000001,
                     0, 0, 0, 1}, true),
                     
    FIDUCIAL_5("apriltag3_36h11_classic", 5, 165.1, 
        new double[]{0.8660254037844387, 0, 0.49999999999999994, 0.5020799999999994,
                     0, 1, 0, -2.1110939999999996,
                     -0.49999999999999994, 0, 0.8660254037844387, 1.8679160000000001,
                     0, 0, 0, 1}, true),
                     
    FIDUCIAL_6("apriltag3_36h11_classic", 6, 165.1, 
        new double[]{0.5000000000000001, 0.8660254037844386, 0, 4.700446000000001,
                     -0.8660254037844386, 0.5000000000000001, 0, -0.7196820000000002,
                     0, 0, 1, 0.308102, 0, 0, 0, 1}, true),
                     
    FIDUCIAL_7("apriltag3_36h11_classic", 7, 165.1, 
        new double[]{1, 0, 0, 5.116498,
                     0, 1, 0, -0.00009999999999976694,
                     0, 0, 1, 0.308102, 0, 0, 0, 1}, true),
                     
    FIDUCIAL_8("apriltag3_36h11_classic", 8, 165.1, 
        new double[]{0.5000000000000001, -0.8660254037844386, 0, 4.700446000000001,
                     0.8660254037844386, 0.5000000000000001, 0, 0.7194820000000002,
                     0, 0, 1, 0.308102, 0, 0, 0, 1}, true),
                     
    FIDUCIAL_9("apriltag3_36h11_classic", 9, 165.1, 
        new double[]{-0.4999999999999998, -0.8660254037844388, 0, 3.869358,
                     0.8660254037844388, -0.4999999999999998, 0, 0.7194820000000002,
                     0, 0, 1, 0.308102, 0, 0, 0, 1}, true),
                     
    FIDUCIAL_10("apriltag3_36h11_classic", 10, 165.1, 
        new double[]{-1, -1.2246467991473532e-16, 0, 3.4533059999999995,
                     1.2246467991473532e-16, -1, 0, -0.00009999999999976694,
                     0, 0, 1, 0.308102, 0, 0, 0, 1}, true),
                     
    FIDUCIAL_11("apriltag3_36h11_classic", 11, 165.1, 
        new double[]{-0.5000000000000002, 0.8660254037844384, 0, 3.869358,
                     -0.8660254037844384, -0.5000000000000002, 0, -0.7196820000000002,
                     0, 0, 1, 0.308102, 0, 0, 0, 1}, true),
                     
    FIDUCIAL_12("apriltag3_36h11_classic", 12, 165.1, 
        new double[]{0.5877852522924731, -0.8090169943749473, 0, -7.922845999999999,
                     0.8090169943749473, 0.5877852522924731, 0, -3.3706799999999997,
                     0, 0, 1, 1.4859, 0, 0, 0, 1}, true),
                     
    FIDUCIAL_13("apriltag3_36h11_classic", 13, 165.1, 
        new double[]{0.587785252292473, 0.8090169943749475, 0, -7.922845999999999,
                     -0.8090169943749475, 0.587785252292473, 0, 3.3704799999999997,
                     0, 0, 1, 1.4859, 0, 0, 0, 1}, true),
                     
    FIDUCIAL_14("apriltag3_36h11_classic", 14, 165.1, 
        new double[]{-0.8660254037844388, -1.2246467991473532e-16, -0.49999999999999994, -0.501728,
                     1.0605752387249069e-16, -1, 6.123233995736766e-17, 2.111656,
                     -0.49999999999999994, 0, 0.8660254037844387, 1.8679160000000001,
                     0, 0, 0, 1}, true),
                     
    FIDUCIAL_15("apriltag3_36h11_classic", 15, 165.1, 
        new double[]{-0.8660254037844388, -1.2246467991473532e-16, -0.49999999999999994, -0.501728,
                     1.0605752387249069e-16, -1, 6.123233995736766e-17, -2.1110939999999996,
                     -0.49999999999999994, 0, 0.8660254037844387, 1.8679160000000001,
                     0, 0, 0, 1}, true),
                     
    FIDUCIAL_16("apriltag3_36h11_classic", 16, 165.1, 
        new double[]{-2.220446049250313e-16, -1.0000000000000002, 0, -2.7864579999999997,
                     1.0000000000000002, -2.220446049250313e-16, 0, -4.0298099999999994,
                     0, 0, 1, 1.30175, 0, 0, 0, 1}, true),
                     
    FIDUCIAL_17("apriltag3_36h11_classic", 17, 165.1, 
        new double[]{-0.5000000000000002, 0.8660254037844384, 0, -4.700094,
                     -0.8660254037844384, -0.5000000000000002, 0, -0.7196820000000002,
                     0, 0, 1, 0.308102, 0, 0, 0, 1}, true),
                     
    FIDUCIAL_18("apriltag3_36h11_classic", 18, 165.1, 
        new double[]{-1, -1.2246467991473532e-16, 0, -5.116399999999999,
                     1.2246467991473532e-16, -1, 0, -0.00009999999999976694,
                     0, 0, 1, 0.308102, 0, 0, 0, 1}, true),
                     
    FIDUCIAL_19("apriltag3_36h11_classic", 19, 165.1, 
        new double[]{-0.4999999999999998, -0.8660254037844388, 0, -4.700094,
                     0.8660254037844388, -0.4999999999999998, 0, 0.7194820000000002,
                     0, 0, 1, 0.308102, 0, 0, 0, 1}, true),
                     
    FIDUCIAL_20("apriltag3_36h11_classic", 20, 165.1, 
        new double[]{0.5000000000000001, -0.8660254037844386, 0, -3.8692599999999997,
                     0.8660254037844386, 0.5000000000000001, 0, 0.7194820000000002,
                     0, 0, 1, 0.308102, 0, 0, 0, 1}, true),
                     
    FIDUCIAL_21("apriltag3_36h11_classic", 21, 165.1, 
        new double[]{1, 0, 0, -3.452953999999999,
                     0, 1, 0, -0.00009999999999976694,
                     0, 0, 1, 0.308102, 0, 0, 0, 1}, true),
                     
    FIDUCIAL_22("apriltag3_36h11_classic", 22, 165.1, 
        new double[]{0.5000000000000001, 0.8660254037844386, 0, -3.8692599999999997,
                     -0.8660254037844386, 0.5000000000000001, 0, -0.7196820000000002,
                     0, 0, 1, 0.308102, 0, 0, 0, 1}, true);

    // Static fields for overall field configuration.
    public static final double FIELD_LENGTH = 17.5482504;
    public static final double FIELD_WIDTH = 8.0519016;
    public static final String TYPE = "frc";

    // Instance fields for each fiducial.
    private final String family;
    private final int id;
    private final double size;
    private final double[] transform;
    private final boolean unique;

    // Constructor for the enum constants.
    Fiducial(String family, int id, double size, double[] transform, boolean unique) {
        this.family = family;
        this.id = id;
        this.size = size;
        this.transform = transform;
        this.unique = unique;
    }

    // Getters
    public String getFamily() {
        return family;
    }

    public int getId() {
        return id;
    }

    public double getSize() {
        return size;
    }

    public double[] getTransform() {
        return transform;
    }

    public boolean isUnique() {
        return unique;
    }

    /**
     * Extracts a WPILib Pose2d from the transform matrix.
     * Assumes that:
     * - x translation is at index 3,
     * - y translation is at index 7,
     * - the rotation is represented by the matrix elements at indices 0 and 4.
     */
    public Pose2d getPose2d() {
        double x = transform[3];
        double y = transform[7];
        // Calculate the rotation angle from the rotation matrix:
        // cos(theta) is at transform[0] and sin(theta) is at transform[4]
        double theta = Math.atan2(transform[4], transform[0]);
        return new Pose2d(new Translation2d(x, y), new Rotation2d(theta));
    }
}
