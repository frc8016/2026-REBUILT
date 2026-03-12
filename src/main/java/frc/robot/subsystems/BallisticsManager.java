package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BallisticsManagerConstants;
import frc.robot.LimelightHelpers;
import java.util.function.Supplier;

public class BallisticsManager extends SubsystemBase {
    private Supplier<Pose3d> targetPoseSupplier;
    private LinearVelocity flywheelVelocity = MetersPerSecond.of(0);
    private Angle hoodAngle = Radians.of(0);
    private Angle targetHorizontalAngle = Radians.of(0);

    public BallisticsManager(Supplier<Pose3d> targetPose) {
        LimelightHelpers.setPipelineIndex("limelight", 0);
        LimelightHelpers.SetIMUMode("limelight", 2);
        this.targetPoseSupplier = targetPose;
    }

    public void update() {
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
