// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class SpindexerConstants {
        public static final double SPINDEXER_SPEED = 1;
    }

    public static class TurretConstants {

        public static final double P_VALUE = 0.0;
        public static final double I_VALUE = 0.0;
        public static final double D_VALUE = 0.0;
        public static final double OUTPUTRANGE_MIN_VALUE = -1.0;
        public static final double OUTPUTRANGE_MAX_VALUE = 1.0;
        public static final double MAX_VEL_RPM = 950;
        public static final double TX_TOLERANCE = 1;
    }

    public static class FeedConstants {
        public static final double FEED_SPEED = 1;
    }

    public static class FlyWheelConstants {
        public static final double IS_READY_DELAY = 0.05;
        public static final double PROPORTIONAL = 0;
        public static final double INTEGRAL = 0;
        public static final double DERIVATIVE = 0;
        public static final double OUTPUT_MIN = -1;
        public static final double OUTPUT_MAX = 1;
        public static final int MAX_CURRENT = 50;
        public static final double IDLE_SETPOINT = 0;
        public static final double SHOOTING_SETPOINT = 1;
        public static final double READY_TOLERANCE = 0.1;
    }

    public static class VisionConstants {
        public static final boolean USE_VISION =
                true; // IMPORTANT we set this to true when useing vision and false
        // when we dont (this will effect all vision uses)
        public static final double VISION_MAX_DIST = 3;
        public static final double MAX_TAG_AMBIGUITY = 0.15;

        public static final String LOWER_RIGHT_CAMERA_NAME = "ArducamOV2311Cam1";
        public static final String LOWER_LEFT_CAMERA_NAME = "ArducamOV2311Cam2";

        public static final Transform3d LOWER_RIGHT_CAMERA_POSE =
                new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(0), // x: forward positive
                                Units.inchesToMeters(0), // y: left positive
                                Units.inchesToMeters(0)), // z: up positive
                        new Rotation3d(
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(0)));

        public static final Transform3d LOWER_LEFT_CAMERA_POSE =
                new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(0), // x: forward positive
                                Units.inchesToMeters(0), // y: left positive
                                Units.inchesToMeters(0)), // z: up positive
                        new Rotation3d(
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(-45)));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout TAG_LAYOUT =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    }
}
