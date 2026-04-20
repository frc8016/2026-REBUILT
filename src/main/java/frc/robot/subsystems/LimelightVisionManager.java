package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.LimelightHelpers;
import java.util.Optional;
import java.util.function.Supplier;

public class LimelightVisionManager extends SubsystemBase {
    private final CommandSwerveDrivetrain drivetrain;

    private final Supplier<Angle> turretAngleSupplier;
    private final Supplier<AngularVelocity> turretAngularVelSupplier;
    private final String limelightName = "limelight";
    private Field2d limelightPose = new Field2d();

    public LimelightVisionManager(
            CommandSwerveDrivetrain drivetrain,
            Supplier<Angle> turretAngleDegSupplier,
            Supplier<AngularVelocity> turretAngularVelDegPerSecSupplier) {

        this.drivetrain = drivetrain;
        this.turretAngleSupplier = turretAngleDegSupplier;
        this.turretAngularVelSupplier = turretAngularVelDegPerSecSupplier;

        LimelightHelpers.setPipelineIndex(limelightName, 0);
        LimelightHelpers.SetIMUMode(limelightName, 1);
        LimelightHelpers.setCameraPose_RobotSpace(
                limelightName,
                LimelightConstants.CAM_FORWARD,
                LimelightConstants.CAM_RIGHT,
                LimelightConstants.CAM_UP,
                LimelightConstants.CAM_ROLL,
                LimelightConstants.CAM_PITCH,
                0);
    }

    public void enableInternalIMU() {
        LimelightHelpers.SetIMUMode(limelightName, 0);
    }

    @Override
    public void periodic() {
        var state = drivetrain.getState();

        // Latency compensation
        Time latency =
                Milliseconds.of(
                        LimelightHelpers.getLatency_Capture(limelightName)
                                + LimelightHelpers.getLatency_Pipeline(limelightName));
        Angle turretAngle = turretAngleSupplier.get();
        AngularVelocity turretVel = turretAngularVelSupplier.get();
        Angle compensatedTurretAngle = compensateForLatency(turretAngle, turretVel, latency);
        Rotation2d turretRotation = Rotation2d.fromDegrees(compensatedTurretAngle.in(Degrees));

        updateLimelightCameraPose(turretRotation);

        double latencyAdjustedTimestamp = Timer.getFPGATimestamp() - latency.in(Seconds);
        Optional<Pose2d> sample = drivetrain.samplePoseAt(latencyAdjustedTimestamp);
        if (sample.isEmpty()) return;
        Rotation2d historicalRotation = sample.get().getRotation();
        double pitch = drivetrain.getPigeon2().getPitch().getValueAsDouble();
        double roll = drivetrain.getPigeon2().getRoll().getValueAsDouble();

        LimelightHelpers.SetRobotOrientation(
                limelightName,
                historicalRotation.getDegrees(),
                // Units.radiansToDegrees(state.Speeds.omegaRadiansPerSecond),
                0,
                pitch,
                0,
                roll,
                0);

        var llEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        if (llEstimate == null || llEstimate.tagCount == 0) return;

        Pose2d llPose = llEstimate.pose;

        // Hard rejects
        double distanceToOdometry =
                state.Pose.getTranslation().getDistance(llPose.getTranslation());
        double omegaDegPerSec = Units.radiansToDegrees(state.Speeds.omegaRadiansPerSecond);
        if (Math.abs(omegaDegPerSec) > 360.0) return;
        if (llEstimate.avgTagDist > 5.0 || llEstimate.avgTagDist < 0.5) return;
        if (distanceToOdometry > 0.5) return;

        // Build Covariance matrix
        double linearSpeed =
                Math.hypot(state.Speeds.vxMetersPerSecond, state.Speeds.vyMetersPerSecond);
        boolean highSpeed = linearSpeed > 3.0;
        boolean highRotation = Math.abs(omegaDegPerSec) > 180.0;
        double poseDiff = state.Pose.getTranslation().getDistance(llPose.getTranslation());
        double xyStdDev = computeXYStdDev(llEstimate, poseDiff, highSpeed, highRotation);
        double thetaStdDev = computeThetaStdDev(llEstimate);

        drivetrain.addVisionMeasurement(
                llPose,
                llEstimate.timestampSeconds,
                VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev));

        limelightPose.setRobotPose(llPose);
        SmartDashboard.putData("limelightPose", limelightPose);
    }

    private Angle compensateForLatency(Angle angle, AngularVelocity vel, Time latency) {
        return angle.minus(vel.times(latency));
    }

    private void updateLimelightCameraPose(Rotation2d turretRotation) {
        // Calculate current camera position relative to robot center
        // Translation = RobotToTurretCenter + (TurretCenterToCamera rotated by turret angle)
        Translation2d camLocation =
                TurretConstants.TURRET_OFFSET.plus(
                        LimelightConstants.CAM_OFFSET_FROM_TURRET_CENTER.rotateBy(turretRotation));

        LimelightHelpers.setCameraPose_RobotSpace(
                limelightName,
                camLocation.getX(),
                -camLocation.getY(),
                LimelightConstants.CAM_UP,
                LimelightConstants.CAM_ROLL,
                LimelightConstants.CAM_PITCH,
                turretRotation.getDegrees());
    }

    private double computeXYStdDev(
            LimelightHelpers.PoseEstimate est,
            double poseDiff,
            boolean highSpeed,
            boolean highRotation) {
        double stdDev = 0.05 + Math.pow(est.avgTagDist, 2) * 0.02;

        if (est.tagCount >= 2) stdDev *= 0.6;
        if (est.tagCount >= 3) stdDev *= 0.4;

        if (poseDiff > 1.0) stdDev *= 1.5;
        if (poseDiff > 2.0) stdDev *= 2.0;

        if (highSpeed) stdDev *= 1.3;
        if (highRotation) stdDev *= 1.3;

        return stdDev;
    }

    private double computeThetaStdDev(LimelightHelpers.PoseEstimate est) {
        if (est.tagCount >= 3) return Units.degreesToRadians(5);
        if (est.tagCount >= 2) return Units.degreesToRadians(10);
        return Units.degreesToRadians(30);
    }
}
