package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BallisticsManagerConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.LimelightHelpers;
import java.util.function.Supplier;

public class BallisticsManager extends SubsystemBase {
    private final Supplier<Pose3d> targetPoseSupplier;
    private final Supplier<Double> robotYawDegreesSupplier;
    private final Supplier<Angle> turretAngleSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    private LinearVelocity flywheelVelocity = MetersPerSecond.of(0);
    private Angle hoodAngle = Radians.of(0);
    private Angle targetHorizontalAngle = Radians.of(0);
    private Field2d turretPoseField = new Field2d();
    private double targetDistanceMeters = 0;

    public BallisticsManager(
            Supplier<Pose3d> targetPose,
            Supplier<Double> robotYawDegrees,
            Supplier<Angle> turretAngle,
            Supplier<ChassisSpeeds> chassisSpeeds) {
        LimelightHelpers.setPipelineIndex("limelight", 0);
        LimelightHelpers.SetIMUMode("limelight", 1);
        // Camera offset from turret pivot (turret is treated as "robot" for the Limelight)
        LimelightHelpers.setCameraPose_RobotSpace(
                "limelight",
                LimelightConstants.CAM_FORWARD,
                LimelightConstants.CAM_RIGHT,
                LimelightConstants.CAM_UP,
                LimelightConstants.CAM_ROLL,
                LimelightConstants.CAM_PITCH,
                0);
        this.targetPoseSupplier = targetPose;
        this.robotYawDegreesSupplier = robotYawDegrees;
        this.turretAngleSupplier = turretAngle;
        this.chassisSpeedsSupplier = chassisSpeeds;
    }

    /** Switch to internal IMU mode once the IMU has been seeded during disabled. */
    public void enableInternalIMU() {
        LimelightHelpers.SetIMUMode("limelight", 3);
    }

    @Override
    public void periodic() {
        SmartDashboard.putData("turretPose", turretPoseField);
        SmartDashboard.putNumber("targetDistanceMeters", targetDistanceMeters);
    }

    public void update() {
        // Turret's field heading = robot heading + turret angle relative to robot
        double turretFieldYaw =
                robotYawDegreesSupplier.get() - turretAngleSupplier.get().in(Degrees);
        LimelightHelpers.SetRobotOrientation("limelight", turretFieldYaw, 0, 0, 0, 0, 0);

        LimelightHelpers.PoseEstimate limelightEstimate =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        if (limelightEstimate == null) return;

        Pose2d turretPose = limelightEstimate.pose;

        // A (0, 0) pose means the limelight has no valid data — skip this cycle
        if (Math.abs(turretPose.getX()) < 1E-6 && Math.abs(turretPose.getY()) < 1E-6) {
            return;
        }

        // Compute turret pivot field velocity from drivetrain odometry + geometry
        ChassisSpeeds speeds = chassisSpeedsSupplier.get();
        double robotHeadingRad = Math.toRadians(robotYawDegreesSupplier.get());
        Translation2d turretFieldVelocity = computeTurretFieldVelocity(speeds, robotHeadingRad);

        // Latency compensation — extrapolate turret pose forward in time
        double latency = limelightEstimate.latency / 1000.0; // Convert ms to seconds
        Translation2d futurePosition =
                turretPose.getTranslation().plus(turretFieldVelocity.times(latency));
        double turretFieldOmega = speeds.omegaRadiansPerSecond;
        Rotation2d futureRotation =
                turretPose.getRotation().plus(new Rotation2d(turretFieldOmega * latency));
        Pose2d futurePose = new Pose2d(futurePosition, futureRotation);

        // Compute field-relative vector from turret to target
        Pose2d target = targetPoseSupplier.get().toPose2d();
        Translation2d toTarget = target.getTranslation().minus(futurePose.getTranslation());
        double targetDistanceMeters = toTarget.getNorm();

        this.targetDistanceMeters = targetDistanceMeters;

        // Avoid zero-length vector which causes NaN in angle calculations
        if (targetDistanceMeters < 1E-6) return;

        double flywheelMps =
                BallisticsManagerConstants.FLYWHEEL_SPEED_MAP.get(targetDistanceMeters);
        double hoodDegrees = BallisticsManagerConstants.HOOD_ANGLE_MAP.get(targetDistanceMeters);
        double hoodRadians = Math.toRadians(hoodDegrees);

        // Build the 3D stationary shot vector in field coordinates.
        // This is the velocity the note needs in field-frame to reach the target.
        double cosHood = Math.cos(hoodRadians);
        double sinHood = Math.sin(hoodRadians);
        Translation2d horizontalDir = toTarget.div(targetDistanceMeters); // unit vector
        Translation3d stationaryShotVec =
                new Translation3d(
                        horizontalDir.getX() * flywheelMps * cosHood,
                        horizontalDir.getY() * flywheelMps * cosHood,
                        flywheelMps * sinHood);
        this.turretPoseField.setRobotPose(turretPose);

        // Velocity correction: the note's field velocity = turret velocity + launch velocity.
        // We want the field velocity to equal stationaryShotVec, so:
        //   launch velocity = stationaryShotVec - turretVelocity
        Translation3d turretVelVec =
                new Translation3d(turretFieldVelocity.getX(), turretFieldVelocity.getY(), 0);
        Translation3d adjustedShotVec = stationaryShotVec.minus(turretVelVec);

        // Extract corrected parameters from the adjusted shot vector
        Translation2d adjustedHorizontal = adjustedShotVec.toTranslation2d();
        double adjustedHorizontalNorm = adjustedHorizontal.getNorm();

        this.flywheelVelocity = MetersPerSecond.of(adjustedShotVec.getNorm());
        this.hoodAngle = Radians.of(Math.atan2(adjustedShotVec.getZ(), adjustedHorizontalNorm));

        // Convert the field-relative shot direction to turret-relative angle
        Rotation2d fieldShotAngle =
                new Rotation2d(adjustedHorizontal.getX(), adjustedHorizontal.getY());
        this.targetHorizontalAngle = fieldShotAngle.minus(futurePose.getRotation()).getMeasure();
    }

    /**
     * Computes the field-relative velocity of the turret pivot from chassis odometry and geometry.
     *
     * <p>v_pivot = v_robot + ω × r_pivot, then rotated into field frame.
     */
    private Translation2d computeTurretFieldVelocity(
            ChassisSpeeds robotSpeeds, double robotHeadingRad) {
        double omega = robotSpeeds.omegaRadiansPerSecond;

        // Robot-relative velocity at turret pivot (WPILib: x = forward, y = left)
        // Cross product ω × r = (-ω * r_y, ω * r_x) for CCW-positive ω
        double vxRobot = robotSpeeds.vxMetersPerSecond - omega * TurretConstants.PIVOT_LEFT_M;
        double vyRobot = robotSpeeds.vyMetersPerSecond + omega * TurretConstants.PIVOT_FORWARD_M;

        // Rotate from robot frame to field frame
        double cos = Math.cos(robotHeadingRad);
        double sin = Math.sin(robotHeadingRad);
        return new Translation2d(vxRobot * cos - vyRobot * sin, vxRobot * sin + vyRobot * cos);
    }

    public Supplier<Angle> TX() {
        return () -> this.targetHorizontalAngle;
    }

    public Supplier<Angle> hoodAngleSupplier() {
        return () -> this.hoodAngle;
    }

    public Supplier<LinearVelocity> flywheelVelocitySupplier() {
        return () -> this.flywheelVelocity;
    }
}
