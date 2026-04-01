package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radian;
import static frc.robot.Constants.BallisticsManagerConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.TurretConstants;
import java.util.function.Supplier;

public class BallisticsManager extends SubsystemBase {

    private final Supplier<Pose3d> targetPoseSupplier;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<Angle> turretAngleSupplier; // CCW negative
    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

    private LinearVelocity flywheelVelocity = MetersPerSecond.of(0);
    private Angle hoodAngle = Degree.of(0);
    private Angle targetHorizontalAngle = Degree.of(0);
    private Field2d turretPoseField = new Field2d();
    private double targetDistanceMeters = 0;
    private double compensatedTargetDistanceMeters = 0;

    // private static TunableNumber flywheelMps = new TunableNumber("flywheelMps", 10);
    // private static TunableNumber hoodDeg = new TunableNumber("hoodDeg", 40);

    public BallisticsManager(
            Supplier<Pose3d> targetPoseSupplier,
            Supplier<Pose2d> robotPoseSupplier,
            Supplier<Angle> turretAngleSupplier,
            Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        this.targetPoseSupplier = targetPoseSupplier;
        this.robotPoseSupplier = robotPoseSupplier;
        this.turretAngleSupplier = turretAngleSupplier;
        this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    }

    @Override
    public void periodic() {
        turretPoseField.setRobotPose(getTurretPose());
        SmartDashboard.putData("turretPose", turretPoseField);
        SmartDashboard.putNumber("targetDistanceMeters", targetDistanceMeters);
    }

    public void update() {
        // Convert target Pose3d to 2D for horizontal targeting
        Pose3d targetPose3d = targetPoseSupplier.get();
        Pose2d targetPose2d =
                new Pose2d(
                        targetPose3d.getX(),
                        targetPose3d.getY(),
                        targetPose3d.getRotation().toRotation2d());

        // // Include turret offset
        Pose2d turretPose = getTurretPose();

        // Compute vector from turret to target in 2D
        Translation2d targetTranslation = targetPose2d.relativeTo(turretPose).getTranslation();
        targetDistanceMeters = targetTranslation.getNorm();

        if (targetDistanceMeters < 1E-6) return;

        // Lookup ballistics tables using horizontal distance
        double flywheelMps = FLYWHEEL_SPEED_MAP.get(targetDistanceMeters);
        double hoodDeg = HOOD_ANGLE_MAP.get(targetDistanceMeters);

        double tof =
                estimateTimeOfFlight(
                        targetDistanceMeters,
                        flywheelMps,
                        Degree.of(hoodDeg),
                        LimelightConstants.CAM_UP,
                        targetPose3d.getZ());

        Translation2d turretFieldVelocity =
                computeTurretFieldVelocity(
                        chassisSpeedsSupplier.get(),
                        robotPoseSupplier.get().getRotation(),
                        turretAngleSupplier.get());

        Translation2d compensatedTargetTranslation =
                targetTranslation.minus(turretFieldVelocity.times(tof));
        compensatedTargetDistanceMeters = compensatedTargetTranslation.getNorm();

        double compensatedFlywheelMps = FLYWHEEL_SPEED_MAP.get(compensatedTargetDistanceMeters);
        double compensatedHoodDeg = HOOD_ANGLE_MAP.get(compensatedTargetDistanceMeters);

        flywheelVelocity = MetersPerSecond.of(compensatedFlywheelMps);
        hoodAngle = Degree.of(compensatedHoodDeg);
        targetHorizontalAngle = compensatedTargetTranslation.getAngle().getMeasure();

        turretPoseField.setRobotPose(turretPose);
    }

    public Supplier<Angle> TX() {
        return () -> targetHorizontalAngle;
    }

    public Supplier<Angle> hoodAngleSupplier() {
        return () -> hoodAngle;
    }

    public Supplier<LinearVelocity> flywheelVelocitySupplier() {
        return () -> flywheelVelocity;
    }

    private double estimateTimeOfFlight(
            double distanceMeters,
            double flywheelMps,
            Angle hoodAngle,
            double turretHeightMeters,
            double targetHeightMeters) {

        double theta = hoodAngle.in(Radian);
        double deltaY = targetHeightMeters - turretHeightMeters;
        double v0 = flywheelMps;
        double g = 9.81;

        double a = 0.5 * g;
        double b = -v0 * Math.sin(theta);
        double c = deltaY;

        double discriminant = b * b - 4 * a * c;
        if (discriminant < 0) {
            // no real solution, fallback
            return distanceMeters / (v0 * Math.cos(theta));
        }

        double t1 = (-b + Math.sqrt(discriminant)) / (2 * a);
        double t2 = (-b - Math.sqrt(discriminant)) / (2 * a);

        double tof = Math.max(t1, t2); // pick positive flight time
        return tof;
    }

    private Pose2d getTurretPose() {
        // Get fresh data inside the lambda
        Pose2d robotPose = robotPoseSupplier.get();
        Rotation2d turretRot = new Rotation2d(-turretAngleSupplier.get().in(Radian));

        // Create the transform (Fixed Offset, Current Rotation)
        Transform2d robotToTurret = new Transform2d(TurretConstants.TURRET_OFFSET, turretRot);

        // Apply to the robot's global pose
        return robotPose.transformBy(robotToTurret);
    }

    private Translation2d computeTurretFieldVelocity(
            ChassisSpeeds chassisSpeeds, Rotation2d robotHeading, Angle turretAngle) {
        // Rotate pivot offset by turret angle (CCW negative)
        Rotation2d turretRotation = Rotation2d.fromDegrees(-turretAngle.in(Degrees));
        Translation2d rotatedOffset = TurretConstants.TURRET_OFFSET.rotateBy(turretRotation);

        // Angular velocity contribution at pivot
        double omega = chassisSpeeds.omegaRadiansPerSecond;
        double vxRot = -omega * rotatedOffset.getY(); // -ω * y
        double vyRot = omega * rotatedOffset.getX(); // ω * x

        // Robot-relative velocity at turret
        double vxRobot = chassisSpeeds.vxMetersPerSecond + vxRot;
        double vyRobot = chassisSpeeds.vyMetersPerSecond + vyRot;

        // Rotate to field frame
        double cos = robotHeading.getCos();
        double sin = robotHeading.getSin();
        double vxField = vxRobot * cos - vyRobot * sin;
        double vyField = vxRobot * sin + vyRobot * cos;
        return new Translation2d(vxField, vyField);
    }
}
