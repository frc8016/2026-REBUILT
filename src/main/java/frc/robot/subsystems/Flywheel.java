package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FlyWheelConstants;

public class Flywheel extends SubsystemBase {
    private boolean isReady() {
        return false; // TODO: create logic to tell if ready
    }

    public Command spinFlywheel() {
        return null;
    }

    public final Trigger isReady =
            new Trigger(this::isReady)
                    // Stay ready for short time after to prevent flapping
                    .debounce(FlyWheelConstants.IS_READY_DELAY, Debouncer.DebounceType.kFalling);
}
