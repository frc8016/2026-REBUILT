package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BallisticsManagerConstants;
import frc.robot.Constants.TurretConstants;
import java.util.function.Supplier;

public class BallisticsManager extends SubsystemBase {

    private final Supplier<Pose3d> targetPoseSupplier;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final Supplier<Angle> turretAngleSupplier; // CCW negative

    private LinearVelocity flywheelVelocity = MetersPerSecond.of(0);
    private Angle hoodAngle = Degree.of(0);
    private Angle targetHorizontalAngle = Degree.of(0);
    private Field2d turretPoseField = new Field2d();
    private double targetDistanceMeters = 0;

    public BallisticsManager(
            Supplier<Pose3d> targetPoseSupplier,
            Supplier<Pose2d> robotPoseSupplier,
            Supplier<Angle> turretAngleSupplier) {
        this.targetPoseSupplier = targetPoseSupplier;
        this.robotPoseSupplier = robotPoseSupplier;
        this.turretAngleSupplier = turretAngleSupplier;
    }

    @Override
    public void periodic() {
        turretPoseField.setRobotPose(getTurretPose().get());
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
        Pose2d turretPose = getTurretPose().get();

        // Compute vector from turret to target in 2D
        Translation2d targetTranslation =
                targetPose2d.getTranslation().minus(turretPose.getTranslation());

        targetDistanceMeters = targetTranslation.getNorm();

        if (targetDistanceMeters < 1E-6) return;

        // Lookup ballistics tables using horizontal distance
        double flywheelMps =
                BallisticsManagerConstants.FLYWHEEL_SPEED_MAP.get(targetDistanceMeters);
        double hoodDeg = BallisticsManagerConstants.HOOD_ANGLE_MAP.get(targetDistanceMeters);

        flywheelVelocity = MetersPerSecond.of(flywheelMps);
        hoodAngle = Degree.of(hoodDeg);
        targetHorizontalAngle = targetTranslation.getAngle().getMeasure();

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

    private Supplier<Pose2d> getTurretPose() {
        return () -> {
            // Get fresh data inside the lambda
            Pose2d robotPose = robotPoseSupplier.get();
            Rotation2d turretRot = new Rotation2d(-turretAngleSupplier.get().in(Degrees));

            // Create the transform (Fixed Offset, Current Rotation)
            Transform2d robotToTurret = new Transform2d(TurretConstants.TURRET_OFFSET, turretRot);

            // Apply to the robot's global pose
            return robotPose.transformBy(robotToTurret);
        };
    }
}
