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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BallisticsManagerConstants;
import frc.robot.Constants.TurretConstants;
import java.util.function.Supplier;

public class BallisticsManager extends SubsystemBase {

    private final Supplier<Pose3d> targetPoseSupplier;
    private final Supplier<Pose2d> robotPoseSupplier;

    private LinearVelocity flywheelVelocity = MetersPerSecond.of(0);
    private Angle hoodAngle = Degree.of(0);
    private Angle targetHorizontalAngle = Degree.of(0);
    private double targetDistanceMeters = 0;
    private double tempTargetHorizontalAngle = 0;

    // private static TunableNumber flywheelMps = new TunableNumber("flywheelMps", 10);
    // private static TunableNumber hoodDeg = new TunableNumber("hoodDeg", 40);

    public BallisticsManager(
            Supplier<Pose3d> targetPoseSupplier, Supplier<Pose2d> robotPoseSupplier) {
        this.targetPoseSupplier = targetPoseSupplier;
        this.robotPoseSupplier = robotPoseSupplier;
    }

    @Override
    public void periodic() {
        // turretPoseField.setRobotPose(getTargetHorizontalAngle().get());
        // SmartDashboard.putData("turretPose", turretPoseField);
        SmartDashboard.putNumber("targetDistanceMeters", targetDistanceMeters);
        SmartDashboard.putNumber("targetHorizontalAngle", tempTargetHorizontalAngle);
    }

    public void update() {
        Translation2d targetTranslation = getTargetHorizontalTranslation().get();
        tempTargetHorizontalAngle = targetTranslation.getAngle().getDegrees();

        targetDistanceMeters = targetTranslation.getNorm();

        if (targetDistanceMeters < 1E-6) return;

        targetHorizontalAngle = Degrees.of(tempTargetHorizontalAngle);

        // Lookup ballistics tables using horizontal distance
        double flywheelMps =
                BallisticsManagerConstants.FLYWHEEL_SPEED_MAP.get(targetDistanceMeters);
        double hoodDeg = BallisticsManagerConstants.HOOD_ANGLE_MAP.get(targetDistanceMeters);

        flywheelVelocity = MetersPerSecond.of(flywheelMps);
        hoodAngle = Degree.of(hoodDeg);

        // targetHorizontalAngle = targetTranslation.getAngle().getMeasure();

        // turretPoseField.setRobotPose(turretPose);
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

    private Supplier<Translation2d> getTargetHorizontalTranslation() {
        return () -> {
            // Convert target Pose3d to 2D for horizontal targeting
            Pose3d targetPose3d = targetPoseSupplier.get();
            Pose2d targetPose2d =
                    new Pose2d(
                            targetPose3d.getX(),
                            targetPose3d.getY(),
                            targetPose3d.getRotation().toRotation2d());

            // Get fresh data inside the lambda
            Pose2d robotPose = robotPoseSupplier.get();

            // Create the transform (Fixed Offset, Current Rotation)
            Transform2d robotToTurret =
                    new Transform2d(TurretConstants.TURRET_OFFSET, new Rotation2d());

            // Include turret offset
            Pose2d turretForwardPose = robotPose.transformBy(robotToTurret);

            // Compute vector from turret to target in 2D
            Translation2d targetTranslation =
                    targetPose2d.relativeTo(turretForwardPose).getTranslation();

            // Apply to the robot's global pose
            return targetTranslation;
        };
    }
}
