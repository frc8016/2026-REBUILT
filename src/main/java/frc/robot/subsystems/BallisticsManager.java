package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BallisticsManagerConstants;
import frc.robot.LimelightHelpers;
import java.util.function.Supplier;

public class BallisticsManager extends SubsystemBase {
    private Supplier<Pose3d> targetPoseSupplier;
    private LinearVelocity flywheelVelocity = MetersPerSecond.of(0);
    private Rotation2d hoodAngle = new Rotation2d();
    private Rotation2d targetHorizontalAngle = new Rotation2d();

    public BallisticsManager(Supplier<Pose3d> targetPose) {
        LimelightHelpers.setPipelineIndex("limelight", 0);
        LimelightHelpers.SetIMUMode("limelight", 2);
        this.targetPoseSupplier = targetPose;
    }

    public void update() {
        Pose2d turretPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight").pose;
        Pose2d target = targetPoseSupplier.get().toPose2d();
        Translation2d targetTranslation = target.relativeTo(turretPose).getTranslation();

        double targetDistanceMeters = targetTranslation.getNorm();
        double flywheelMps = computeTargetProjectileVelocity(targetDistanceMeters);
        double zMeters = targetPoseSupplier.get().getZ();
        double hoodRadians = computeHoodAngle(targetDistanceMeters, flywheelMps, zMeters);

        this.flywheelVelocity = MetersPerSecond.of(flywheelMps);
        this.hoodAngle = new Rotation2d(hoodRadians);
        this.targetHorizontalAngle = targetTranslation.getAngle();
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

        if (discriminant < 0) return 45.0;

        return Math.atan((v2 + Math.sqrt(discriminant)) / (g * d));
    }

    public Supplier<Rotation2d> TX() {
        return () -> this.targetHorizontalAngle;
    }

    public Supplier<Rotation2d> hoodAngleSupplier() {
        return () -> this.hoodAngle;
    }

    public Supplier<LinearVelocity> flywheelVelocitySupplier() {
        return () -> this.flywheelVelocity;
    }
}
