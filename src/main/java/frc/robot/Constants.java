// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Time;
import java.util.HashMap;
import java.util.Map;

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
        public static final double SPINDEXER_SPEED = -1;
    }

    public static class TurretConstants {

        public static final double P_VALUE = 0.0; // find
        public static final double I_VALUE = 0.0; // find
        public static final double D_VALUE = 0.0; // find
        public static final double OUTPUTRANGE_MIN_VALUE = -1.0;
        public static final double OUTPUTRANGE_MAX_VALUE = 1.0;
        public static final double MAX_VEL_RPM = 950;
        public static final double TX_TOLERANCE = 1;
        public static final int MAX_CURRENT = 50;
        public static final double READY_TOLERANCE = 1;
        public static final double FEED_FORWARD_KS = 0.28;
        public static final double FEED_FORWARD_KV = 1.52;
        public static final double FEED_FORWARD_KA = 0.175;
        public static final double SIM_FEED_FORWARD_KS = 0.044289;
        public static final double SIM_FEED_FORWARD_KV = 0.1227;
        public static final double SIM_FEED_FORWARD_KA = 0.006877;
        public static final AngularVelocity SOFT_LIMIT = RPM.of(5000);
        public static final AngularVelocity TRAPAZOIDAL_MAX_VELOCITY = RPM.of(5000);
        public static final AngularAcceleration TRAPAZOIDAL_MAX_ACCELERATION =
                RotationsPerSecondPerSecond.of(2500);
        public static final Time CLOSED_LOOP_RAMP_RATE = Seconds.of(0.25);
        public static final Time OPEN_LOOP_RAMP_RATE = Seconds.of(0.25);
        public static final Current STATOR_CURRENT_LIMIT = Amps.of(40);
        public static final double IS_READY_DELAY = 0.05;
        public static final Angle BOTTOM_SOFT_LIMIT = Degrees.of(70); // fix
        public static final Angle TOP_SOFT_LIMIT = Degrees.of(30); // fix
        public static final Distance TURRET_LENGTH = Inches.of(17.8); // fix
        public static final Angle START_ANGLE = Degrees.of(0);
        public static final Mass TURRET_WEIGHT = Kilograms.of(10); // fix
    }

    public static class FeedConstants {
        public static final double FEED_SPEED = 1;
    }

    public static class IntakeConstants {}

    public static class ArmConstants {
        public static final Mass MASS = Kilograms.of(3.1);
        public static final Distance ARM_LENGTH = Inches.of(11.25);
        public static final Angle DOWN_ANGLE = Degrees.of(-5);
        public static final Angle UP_ANGLE = Degrees.of(88);
        public static final Angle START_ANGLE = Degrees.of(90);
        public static final Current CURRENT_LIMIT = Amps.of(40);
        public static final Time RAMP_RATE = Seconds.of(0.25);
        public static final Angle TOGGLE_TOLERANCE = Degrees.of(5);

        public static final double PROPORTIONAL = 35;
        public static final double INTEGRAL = 0;
        public static final double DERIVATIVE = 0;

        public static final double KS = 0.98761;
        public static final double KG = 0;
        // public static final double KG = 0.26097;
        public static final double KV = 1.3535;
        public static final double KA = 0.62598;

        // public static final double KS = 0;
        // public static final double KG = 0;
        // public static final double KV = 0;
        // public static final double KA = 0;

        // public static final double KS = 0.047102;
        // public static final double KG = 0.26097;
        // public static final double KV = 4.7211;
        // public static final double KA = 0.63003;

        public static final double SIM_KS = 0;
        public static final double SIM_KG = 0.01;
        public static final double SIM_KV = 0.01;

        public static final Angle SOFT_LIMIT_LOWER = Degrees.of(-10);
        public static final Angle SOFT_LIMIT_UPPER = Degrees.of(89);

        public static final Angle HARD_LIMIT_LOWER = Degrees.of(-10);
        public static final Angle HARD_LIMIT_UPPER = Degrees.of(90);
    }

    public static class BottomFlyWheelConstants {
        public static final double IS_READY_DELAY = 0.05;
        public static final double PROPORTIONAL = 0.015;
        public static final double INTEGRAL = 0;
        public static final double DERIVATIVE = 0;
        public static final int MAX_CURRENT = 50;
        public static final LinearVelocity IDLE_SETPOINT = MetersPerSecond.of(1);
        public static final double READY_TOLERANCE = 3;
        public static final double FEED_FORWARD_KS = 0.61865;
        public static final double FEED_FORWARD_KV = 0.095313;
        public static final double FEED_FORWARD_KA = 0.040166;
        public static final double SIM_FEED_FORWARD_KS = 0.0096372;
        public static final double SIM_FEED_FORWARD_KV = 0.12421;
        public static final double SIM_FEED_FORWARD_KA = 0.15227;
        public static final Distance FLYWHEEL_DIAMETER = Inches.of(4);
        public static final Mass FLYWHEEL_MASS = Kilograms.of(1.531);
        public static final AngularVelocity SOFT_LIMIT = RPM.of(5000);
        public static final AngularVelocity TRAPAZOIDAL_MAX_VELOCITY = RPM.of(5000);
        public static final AngularAcceleration TRAPAZOIDAL_MAX_ACCELERATION =
                RotationsPerSecondPerSecond.of(2500);
        public static final Time CLOSED_LOOP_RAMP_RATE = Seconds.of(0.25);
        public static final Time OPEN_LOOP_RAMP_RATE = Seconds.of(0.25);
        public static final Current STATOR_CURRENT_LIMIT = Amps.of(40);
    }

    public static class TopFlyWheelConstants {
        public static final double PROPORTIONALITY_TO_BOTTOM_FLYWHEEL = 1;
        public static final double IS_READY_DELAY = 0.03;
        public static final double PROPORTIONAL = 0.02;
        public static final double INTEGRAL = 0;
        public static final double DERIVATIVE = 0;
        public static final int MAX_CURRENT = 50;
        public static final LinearVelocity IDLE_SETPOINT = MetersPerSecond.of(1);
        public static final double READY_TOLERANCE = 3;
        public static final double FEED_FORWARD_KS = 0.41484;
        public static final double FEED_FORWARD_KV = 0.10539;
        public static final double FEED_FORWARD_KA = 0.018782;
        public static final double SIM_FEED_FORWARD_KS = 0.044289;
        public static final double SIM_FEED_FORWARD_KV = 0.1227;
        public static final double SIM_FEED_FORWARD_KA = 0.006877;
        public static final Distance FLYWHEEL_DIAMETER = Inches.of(2);
        public static final Mass FLYWHEEL_MASS =
                Kilograms.of(0.25); // TODO: need weight of flywheel
        public static final AngularVelocity SOFT_LIMIT = RPM.of(5000);
        public static final AngularVelocity TRAPAZOIDAL_MAX_VELOCITY = RPM.of(5000);
        public static final AngularAcceleration TRAPAZOIDAL_MAX_ACCELERATION =
                RotationsPerSecondPerSecond.of(2500);
        public static final Time CLOSED_LOOP_RAMP_RATE = Seconds.of(0.25);
        public static final Time OPEN_LOOP_RAMP_RATE = Seconds.of(0.25);
        public static final Current STATOR_CURRENT_LIMIT = Amps.of(40);
    }

    public static class TargetConstants {
        public static final Distance FIELD_LENGTH = Meters.of(Units.inchesToMeters(651.22));
        public static final Distance FIELD_HEIGHT = Meters.of(Units.inchesToMeters(317.69));
        public static final Distance DRIVERSTATION_TO_TRENCH =
                Meters.of(Units.inchesToMeters(156.06));
        public static final Distance ROBOT_WIDTH_WITH_BUMPERS =
                Meters.of(
                        Units.inchesToMeters(33)); // TODO: update constant with actual robot width
        public static final Pose3d HUB_TARGET_BLUE =
                new Pose3d(
                        Units.inchesToMeters(182.11),
                        Units.inchesToMeters(158.84),
                        Units.inchesToMeters(57),
                        new Rotation3d());
        public static final Pose3d FERRY_TARGET_BLUE_LOWER =
                new Pose3d(
                        Units.inchesToMeters(91.055),
                        Units.inchesToMeters(79.4225),
                        Units.inchesToMeters(0),
                        new Rotation3d());
    }

    public static class BallisticsManagerConstants {
        public static final LinearAcceleration G = MetersPerSecondPerSecond.of(9.806);
        public static final double VELOCITY_SLOPE = 0.5;
        public static final LinearVelocity VELOCITY_INTERCEPT = MetersPerSecond.of(7.5);
        public static final LinearVelocity MAX_PROJECTILE_VELOCITY = MetersPerSecond.of(13.0);
        public static final LinearVelocity MIN_PROJECTILE_VELOCITY = MetersPerSecond.of(7.0);
    }

    public static class HoodConstants { // TODO: upadate all constants here
        public static final double PROPORTIONAL = 0.01;
        public static final double INTEGRAL = 0;
        public static final double DERIVATIVE = 0;
        public static final int MAX_CURRENT = 50;
        public static final double READY_TOLERANCE = 1;
        public static final double FEED_FORWARD_KS = 0.5633;
        public static final double FEED_FORWARD_KV = 0;
        public static final double FEED_FORWARD_KA = 1.5975;
        public static final double SIM_FEED_FORWARD_KS = 0.044289;
        public static final double SIM_FEED_FORWARD_KV = 0.1227;
        public static final double SIM_FEED_FORWARD_KA = 0.006877;
        public static final AngularVelocity SOFT_LIMIT = RPM.of(5000);
        public static final AngularVelocity TRAPAZOIDAL_MAX_VELOCITY = RPM.of(5000);
        public static final AngularAcceleration TRAPAZOIDAL_MAX_ACCELERATION =
                RotationsPerSecondPerSecond.of(2500);
        public static final Time CLOSED_LOOP_RAMP_RATE = Seconds.of(0.25);
        public static final Time OPEN_LOOP_RAMP_RATE = Seconds.of(0.25);
        public static final Current STATOR_CURRENT_LIMIT = Amps.of(40);
        public static final double IS_READY_DELAY = 0.05;
        public static final Angle BOTTOM_SOFT_LIMIT = Degrees.of(25); // actual 23
        public static final Angle TOP_SOFT_LIMIT = Degrees.of(46); // actual 48
        public static final Distance HOOD_LENGTH = Inches.of(17.8);
        public static final Angle START_ANGLE = Degrees.of(23);
        public static final Mass HOOD_WEIGHT = Kilograms.of(1);
    }

    public static class VisionConstants {
        public static final boolean USE_VISION =
                true; // IMPORTANT we set this to true when useing vision and false
        // when we dont (this will effect all vision uses)
        public static final double VISION_MAX_DIST = 3;
        public static final double MAX_TAG_AMBIGUITY = 0.15;

        public static final String LOWER_RIGHT_CAMERA_NAME = "ArducamOV2311Cam1";
        public static final String LOWER_LEFT_CAMERA_NAME = "ArducamOV2311Cam2";

        public static final Transform3d LEFT_CAMERA_POSE =
                new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(3), // x: forward positive
                                Units.inchesToMeters(-11.25), // y: left positive
                                Units.inchesToMeters(17)), // z: up positive
                        new Rotation3d(
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(-15),
                                Units.degreesToRadians(-90)));

        public static final Transform3d RIGHT_CAMERA_POSE =
                new Transform3d(
                        new Translation3d(
                                Units.inchesToMeters(3), // x: forward positive
                                Units.inchesToMeters(11.25), // y: left positive
                                Units.inchesToMeters(17)), // z: up positive
                        new Rotation3d(
                                Units.degreesToRadians(0),
                                Units.degreesToRadians(-32),
                                Units.degreesToRadians(90)));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout TAG_LAYOUT =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    }

    public static class AutonomousClimbConstants {
        public static final Map<Pose2d, String> BLUE_CLIMB_POSITIONS =
                new HashMap<>(14) {
                    {
                        put(new Pose2d(1.697, 5.188, new Rotation2d()), "left");
                        put(new Pose2d(1.697, 2.099, new Rotation2d()), "right");
                    }
                };

        public static final Map<Pose2d, String> RED_CLIMB_POSITIONS =
                new HashMap<>(14) {
                    {
                        put(new Pose2d(14.844, 2.099, new Rotation2d()), "left");
                        put(new Pose2d(14.844, 5.188, new Rotation2d()), "right");
                    }
                };
        public static PathConstraints constraints =
                new PathConstraints(
                        5.210, 7.1, Units.degreesToRadians(540), Units.degreesToRadians(1851));
    }
}
