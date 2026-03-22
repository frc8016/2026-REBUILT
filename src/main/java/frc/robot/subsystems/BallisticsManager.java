package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

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
    // private final TunableNumber flywheelMps = new TunableNumber("flywheelMps", 1);
    // private final TunableNumber hoodDegrees = new TunableNumber("hoodDegrees", 40);
    private double targetDistanceMeters = 0;

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
        SmartDashboard.putNumber("targetDistanceMeters", targetDistanceMeters);
    }

    public void update() {
        // Turret's field heading = robot heading + turret angle relative to robot
        double turretFieldYaw =
                robotYawDegreesSupplier.get() + turretAngleSupplier.get().in(Degrees);
        LimelightHelpers.SetRobotOrientation("limelight", turretFieldYaw, 0, 0, 0, 0, 0);

        LimelightHelpers.PoseEstimate limelightEstimate =
                LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

        if (limelightEstimate == null) return;
        Pose2d turretPose = limelightEstimate.pose;

        // A (0, 0) pose means the limelight has no valid data — skip this cycle
        if (Math.abs(turretPose.getX()) < 1E-6 && Math.abs(turretPose.getY()) < 1E-6) {
            return;
        }

        Pose2d target = targetPoseSupplier.get().toPose2d();
        Translation2d targetTranslation = target.relativeTo(turretPose).getTranslation();

        this.targetDistanceMeters = targetTranslation.getNorm();

        // Avoid zero-length Translation2d which causes NaN in getAngle()
        if (targetDistanceMeters < 1E-6) return;

        double flywheelMps =
                BallisticsManagerConstants.FLYWHEEL_SPEED_MAP.get(targetDistanceMeters);
        double hoodDegrees = BallisticsManagerConstants.HOOD_ANGLE_MAP.get(targetDistanceMeters);

        this.flywheelVelocity = MetersPerSecond.of(flywheelMps);
        this.hoodAngle = Degrees.of(hoodDegrees);
        this.targetHorizontalAngle = targetTranslation.getAngle().getMeasure();
        this.turretPoseField.setRobotPose(turretPose);
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
