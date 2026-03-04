package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TargetConstants;
import java.util.Optional;
import java.util.function.Supplier;

public class TargetSelector extends SubsystemBase {
    private Supplier<Pose2d> swervePoseSupplier;
    private Optional<Alliance> alliance = DriverStation.getAlliance();
    private Pose3d target =
            new Pose3d(); // output target updates every time updateTargetSelection() is called

    public TargetSelector(Supplier<Pose2d> poseSupplierIn) {
        swervePoseSupplier = poseSupplierIn;
    }

    public void updateTargetSelection() { // updates target Pose2d
        Pose2d swervePose = swervePoseSupplier.get();
        Distance x = Meters.of(swervePose.getX()); // distance x of swerve
        Distance y = Meters.of(swervePose.getY()); // distance y of swerve
        Pose3d hubTarget = TargetConstants.HUB_TARGET_BLUE;
        Pose3d ferryTargetLower = TargetConstants.FERRY_TARGET_BLUE_LOWER;

        // this flips the blue Pose2ds for red side and flips swerve pose
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            x = TargetConstants.FIELD_LENGTH.minus(x);
            y = TargetConstants.FIELD_HEIGHT.minus(y);
            hubTarget =
                    new Pose3d(
                            TargetConstants.FIELD_LENGTH.in(Meters) - hubTarget.getX(),
                            hubTarget.getY(),
                            hubTarget.getZ(),
                            new Rotation3d());
            ferryTargetLower =
                    new Pose3d(
                            TargetConstants.FIELD_LENGTH.in(Meters) - ferryTargetLower.getX(),
                            TargetConstants.FIELD_HEIGHT.in(Meters) - ferryTargetLower.getY(),
                            ferryTargetLower.getZ(),
                            new Rotation3d());
        }

        if (alliance.isPresent()) { // safety to make sure there is an alliance
            if (x.lte(
                    TargetConstants.DRIVERSTATION_TO_TRENCH.plus(
                            TargetConstants.ROBOT_WIDTH_WITH_BUMPERS.div(
                                    2)))) { // if robot is in home area
                target = hubTarget;
            } else { // if robot is anywhere else
                if (y.lte(TargetConstants.FIELD_HEIGHT.div(2))) { // if robot is on bottom of field
                    target = ferryTargetLower;
                } else { // if robot is on top of field
                    double yFlip =
                            TargetConstants.FIELD_HEIGHT.in(Meters) - ferryTargetLower.getY();
                    // this switches the lower Pose2d for the ferry to be upper
                    target =
                            new Pose3d(
                                    ferryTargetLower.getX(),
                                    yFlip,
                                    ferryTargetLower.getZ(),
                                    new Rotation3d());
                }
            }
        }
    }

    public Supplier<Pose3d> getCurrentTarget() {
        return () -> target;
    }

    public void updateAlliance() {
        if (DriverStation.getAlliance().isPresent()) {
            alliance = DriverStation.getAlliance();
        }
    }
}
