package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FlyWheelConstants;

public class Flywheel extends SubsystemBase {

    private final SparkMax flywheel = new SparkMax(13, MotorType.kBrushless);
    private final SparkMaxConfig flywheelConfig = new SparkMaxConfig();

    private boolean isReady() {
        return MathUtil.isNear(
                FlyWheelConstants.SHOOTING_SETPOINT,
                flywheel.getEncoder().getVelocity(),
                FlyWheelConstants.READY_TOLERANCE);
    }

    public Flywheel() {
        flywheelConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(FlyWheelConstants.PROPORTIONAL, ClosedLoopSlot.kSlot0)
                .i(FlyWheelConstants.INTEGRAL, ClosedLoopSlot.kSlot0)
                .d(FlyWheelConstants.DERIVATIVE, ClosedLoopSlot.kSlot0)
                .outputRange(FlyWheelConstants.OUTPUT_MAX, FlyWheelConstants.OUTPUT_MIN);

        flywheelConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(FlyWheelConstants.MAX_CURRENT);

        flywheel.configure(
                flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public Command spinFlywheel() {
        return new StartEndCommand(
                () ->
                        this.flywheel
                                .getClosedLoopController()
                                .setSetpoint(
                                        FlyWheelConstants.SHOOTING_SETPOINT,
                                        ControlType.kMAXMotionVelocityControl),
                () ->
                        this.flywheel
                                .getClosedLoopController()
                                .setSetpoint(
                                        FlyWheelConstants.IDLE_SETPOINT,
                                        ControlType.kMAXMotionVelocityControl),
                this);
    }

    public final Trigger isReady =
            new Trigger(this::isReady)
                    // Stay ready for short time after to prevent flapping
                    .debounce(FlyWheelConstants.IS_READY_DELAY, Debouncer.DebounceType.kFalling);
}
