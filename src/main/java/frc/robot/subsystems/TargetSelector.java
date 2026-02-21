package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import java.util.Optional;

public class TargetSelector {
    private CommandSwerveDrivetrain drivetrain;
    private Optional<Alliance> alliance = DriverStation.getAlliance();
    private Pose2d target = new Pose2d();

    public TargetSelector(CommandSwerveDrivetrain swerveDrivetrain) {
        drivetrain = swerveDrivetrain;
    }

    public void updateTargetSelection() {
        Distance x = Meters.of(drivetrain.getState().Pose.getX());
        Distance y = Meters.of(drivetrain.getState().Pose.getY());
        Pose2d hubTarget = Constants.FieldZoneLines.HUB_TARGET_BLUE;
        Pose2d ferryTargetLower = Constants.FieldZoneLines.FERRY_TARGET_BLUE_LOWER;

        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            x = Constants.FieldZoneLines.FIELD_LENGTH.minus(x);
            y = Constants.FieldZoneLines.FIELD_HEIGHT.minus(y);
            hubTarget =
                    new Pose2d(
                            Constants.FieldZoneLines.FIELD_LENGTH.in(Meters) - hubTarget.getX(),
                            hubTarget.getY(),
                            new Rotation2d());
            ferryTargetLower =
                    new Pose2d(
                            Constants.FieldZoneLines.FIELD_LENGTH.in(Meters)
                                    - ferryTargetLower.getX(),
                            Constants.FieldZoneLines.FIELD_HEIGHT.in(Meters)
                                    - ferryTargetLower.getY(),
                            new Rotation2d());
        }

        if (alliance.isPresent()) {
            if (x.lte(Constants.FieldZoneLines.DRIVERSTATION_TO_TRENCH)) {
                target = hubTarget;
            } else {
                if (y.lte(Constants.FieldZoneLines.FIELD_HEIGHT.div(2))) {
                    target = ferryTargetLower;
                } else {
                    target =
                            new Pose2d(
                                    ferryTargetLower.getX(),
                                    Constants.FieldZoneLines.FIELD_HEIGHT.in(Meters)
                                            - ferryTargetLower.getY(),
                                    new Rotation2d());
                }
            }
        }
    }

    public Pose2d getCurrentTarget() {
        return target;
    }

    public void updateAlliance() {
        if (DriverStation.getAlliance().isPresent()) {
            alliance = DriverStation.getAlliance();
        }
    }
}
