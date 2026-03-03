package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BallisticsManagerConstants;
import frc.robot.LimelightHelpers;
import java.util.function.Supplier;

public class BallisticsManager extends SubsystemBase {
    private Supplier<Pose3d> targetPoseSupplier;
    private double targetDistance;
    private double flywheelVelocity;
    private double hoodAngle;
    private double targetHorizontalAngle;

    public BallisticsManager(Supplier<Pose3d> targetPose) {
        LimelightHelpers.setPipelineIndex("limelight", 0);
        LimelightHelpers.SetIMUMode("limelight", 2);
        this.targetPoseSupplier = targetPose;
    }

    public void update() {
        Pose2d turretPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight").pose;
        Pose2d target = targetPoseSupplier.get().toPose2d();
        Translation2d targetTranslation = target.relativeTo(turretPose).getTranslation();

        this.targetDistance = targetTranslation.getNorm();
        this.flywheelVelocity = computeTargetProjectileVelocity(this.targetDistance);
        this.hoodAngle =
                computeHoodAngle(
                        this.targetDistance,
                        this.flywheelVelocity,
                        targetPoseSupplier.get().getZ());
    }

    public double computeTargetProjectileVelocity(double d) {
        double velocity =
                (d * BallisticsManagerConstants.VELOCITY_SLOPE)
                        + BallisticsManagerConstants.VELOCITY_INTERCEPT;
        return MathUtil.clamp(
                velocity,
                BallisticsManagerConstants.MIN_PROJECTILE_VELOCITY,
                BallisticsManagerConstants.MAX_PROJECTILE_VELOCITY);
    }

    private double computeHoodAngle(double d, double v, double z) {
        double v2 = v * v;
        double v4 = v2 * v2;
        double g = BallisticsManagerConstants.G;

        double discriminant =
                v4 - g * (g * (this.targetDistance * this.targetDistance) + 2 * z * v2);

        if (discriminant < 0) return 45.0;

        double thetaRadians = Math.atan((v2 - Math.sqrt(discriminant)) / (g * this.targetDistance));
        return Math.toDegrees(thetaRadians);
    }

    public Supplier<Double> TX() {
        return () -> this.targetHorizontalAngle;
    }

    public Supplier<Double> hoodAngleSupplier() {
        return () -> this.hoodAngle;
    }

    public Supplier<Double> flywheelVelocitySupplier() {
        return () -> this.flywheelVelocity;
    }
}
