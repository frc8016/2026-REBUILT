package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.BottomFlyWheelConstants;
import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class BottomFlywheel extends SubsystemBase {

    private final SparkMax flywheelMotor = new SparkMax(4, MotorType.kBrushless);
    private final SparkMax flywheelMotorfollower = new SparkMax(5, MotorType.kBrushless);

    private final SmartMotorControllerConfig motorConfig =
            new SmartMotorControllerConfig(this)
                    .withClosedLoopController(
                            BottomFlyWheelConstants.PROPORTIONAL,
                            BottomFlyWheelConstants.INTEGRAL,
                            BottomFlyWheelConstants.DERIVATIVE,
                            BottomFlyWheelConstants.TRAPAZOIDAL_MAX_VELOCITY,
                            BottomFlyWheelConstants.TRAPAZOIDAL_MAX_ACCELERATION)
                    .withGearing(new MechanismGearing(GearBox.fromReductionStages(1, 1)))
                    .withIdleMode(MotorMode.COAST)
                    .withTelemetry("BottomFlywheelMotor", TelemetryVerbosity.HIGH)
                    .withStatorCurrentLimit(BottomFlyWheelConstants.STATOR_CURRENT_LIMIT)
                    .withMotorInverted(false)
                    .withClosedLoopRampRate(BottomFlyWheelConstants.CLOSED_LOOP_RAMP_RATE)
                    .withOpenLoopRampRate(BottomFlyWheelConstants.OPEN_LOOP_RAMP_RATE)
                    .withFeedforward(
                            new SimpleMotorFeedforward(
                                    BottomFlyWheelConstants.FEED_FORWARD_KS,
                                    BottomFlyWheelConstants.FEED_FORWARD_KV,
                                    BottomFlyWheelConstants.FEED_FORWARD_KA))
                    .withSimFeedforward(
                            new SimpleMotorFeedforward(
                                    BottomFlyWheelConstants.SIM_FEED_FORWARD_KS,
                                    BottomFlyWheelConstants.SIM_FEED_FORWARD_KV,
                                    BottomFlyWheelConstants.SIM_FEED_FORWARD_KA))
                    .withControlMode(ControlMode.CLOSED_LOOP)
                    .withFollowers(Pair.of(flywheelMotorfollower, true));

    private final SmartMotorController motor =
            new SparkWrapper(flywheelMotor, DCMotor.getNEO(1), motorConfig);

    private final FlyWheelConfig flywheelConfig =
            new FlyWheelConfig(motor)
                    .withDiameter(BottomFlyWheelConstants.FLYWHEEL_DIAMETER)
                    .withMass(BottomFlyWheelConstants.FLYWHEEL_MASS)
                    .withTelemetry("BottomFlywheelMechanism", TelemetryVerbosity.HIGH)
                    .withSoftLimit(
                            BottomFlyWheelConstants.SOFT_LIMIT.negate(),
                            BottomFlyWheelConstants.SOFT_LIMIT);
    // .withSpeedometerSimulation(RPM.of(7500)); // optional to make graph of velocity not position

    private final FlyWheel flywheel = new FlyWheel(flywheelConfig);

    private boolean isReady() {
        return flywheel.getSpeed()
                .isNear(
                        flywheel.getMechanismSetpointVelocity().orElse(RotationsPerSecond.of(0)),
                        BottomFlyWheelConstants.READY_TOLERANCE);
    }

    public BottomFlywheel() {}

    public Command spinFlywheel(Supplier<LinearVelocity> velocity) {
        return this.flywheel.run(() -> flywheelConfig.getAngularVelocity(velocity.get()));
    }

    public Command idleFlywheel() {
        return this.flywheel.set(0);
        // return this.flywheel.run(
        //         flywheelConfig.getAngularVelocity(BottomFlyWheelConstants.IDLE_SETPOINT));
    }

    public final Trigger isReady =
            new Trigger(this::isReady)
                    // Stay ready for short time after to prevent flapping
                    .debounce(
                            BottomFlyWheelConstants.IS_READY_DELAY,
                            Debouncer.DebounceType.kFalling);

    @Override
    public void periodic() {
        flywheel.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        flywheel.simIterate();
    }

    public Command sysId() {
        return flywheel.sysId(Volts.of(10), Volts.of(1).per(Second), Seconds.of(5));
    }
}
