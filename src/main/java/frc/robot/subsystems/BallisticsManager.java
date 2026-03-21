package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BallisticsManagerConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.LimelightHelpers;
import java.util.function.Supplier;

public class BallisticsManager extends SubsystemBase {
    private final Supplier<Pose3d> targetPoseSupplier;
    private final Supplier<Double> robotYawDegreesSupplier;
    private final Supplier<Angle> turretAngleSupplier;

    private LinearVelocity flywheelVelocity = MetersPerSecond.of(0);
    private Angle hoodAngle = Radians.of(0);
    private Angle targetHorizontalAngle = Radians.of(0);
    private Field2d turretPoseField = new Field2d();

    public BallisticsManager(
            Supplier<Pose3d> targetPose,
            Supplier<Double> robotYawDegrees,
            Supplier<Angle> turretAngle) {
        LimelightHelpers.setPipelineIndex("limelight", 0);
        // Start with EXTERNAL_SEED so the internal IMU calibrates against the drivetrain gyro
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
    }

    /** Switch to internal IMU mode once the IMU has been seeded during disabled. */
    public void enableInternalIMU() {
        LimelightHelpers.SetIMUMode("limelight", 2);
    }

    @Override
    public void periodic() {
        SmartDashboard.putData("turretPose", turretPoseField);
        SmartDashboard.putNumber("targetHorizontalAngle", targetHorizontalAngle.magnitude());
    }

    public void update() {
        // Turret's field heading = robot heading + turret angle relative to robot
        double turretFieldYaw =
                robotYawDegreesSupplier.get() + turretAngleSupplier.get().in(Degrees);
        LimelightHelpers.SetRobotOrientation("limelight", turretFieldYaw, 0, 0, 0, 0, 0);

        LimelightHelpers.PoseEstimate limelightEstimate =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        if (limelightEstimate == null) return;
        Pose2d turretPose = limelightEstimate.pose;

        // A (0, 0) pose means the limelight has no valid data — skip this cycle
        if (Math.abs(turretPose.getX()) < 1E-6 && Math.abs(turretPose.getY()) < 1E-6) {
            return;
        }

        Pose2d target = targetPoseSupplier.get().toPose2d();
        Translation2d targetTranslation = target.relativeTo(turretPose).getTranslation();

        double targetDistanceMeters = targetTranslation.getNorm();

        // Avoid zero-length Translation2d which causes NaN in getAngle()
        if (targetDistanceMeters < 1E-6) return;

        double flywheelMps = computeTargetProjectileVelocity(targetDistanceMeters);
        double zMeters = targetPoseSupplier.get().getZ();
        double hoodRadians = computeHoodAngle(targetDistanceMeters, flywheelMps, zMeters);

        this.flywheelVelocity = MetersPerSecond.of(flywheelMps);
        this.hoodAngle = Radians.of(hoodRadians);
        this.targetHorizontalAngle = targetTranslation.getAngle().getMeasure();
        this.turretPoseField.setRobotPose(turretPose);
    }

    public double computeTargetProjectileVelocity(double dMeters) {
        double velocity =
                (dMeters * BallisticsManagerConstants.VELOCITY_SLOPE)
                        + BallisticsManagerConstants.VELOCITY_INTERCEPT.magnitude();

        return MathUtil.clamp(
                velocity,
                BallisticsManagerConstants.MIN_PROJECTILE_VELOCITY.magnitude(),
                BallisticsManagerConstants.MAX_PROJECTILE_VELOCITY.magnitude());
    }

    private double computeHoodAngle(double d, double v, double z) {
        double v2 = v * v;
        double v4 = v2 * v2;
        double g = BallisticsManagerConstants.G.magnitude();

        double discriminant = v4 - g * (g * (d * d) + 2 * z * v2);

        if (discriminant < 0) return Math.PI / 4;

        return Math.atan((v2 + Math.sqrt(discriminant)) / (g * d));
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
