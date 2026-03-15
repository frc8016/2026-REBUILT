package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.HoodConstants;
import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class Hood extends SubsystemBase {
    private final SparkMax hoodMotor = new SparkMax(9, MotorType.kBrushless);

    private final SmartMotorControllerConfig hoodMotorConfig =
            new SmartMotorControllerConfig(this)
                    .withClosedLoopController(
                            HoodConstants.PROPORTIONAL,
                            HoodConstants.INTEGRAL,
                            HoodConstants.DERIVATIVE)
                    .withGearing(new MechanismGearing(GearBox.fromReductionStages(1246 / 9)))
                    .withExponentialProfile(
                            Volts.of(12),
                            HoodConstants.MAX_VELOCITY,
                            HoodConstants.MAX_ACCELERATION)
                    .withIdleMode(MotorMode.BRAKE)
                    .withTelemetry("Hood", TelemetryVerbosity.HIGH)
                    .withStatorCurrentLimit(HoodConstants.STATOR_CURRENT_LIMIT)
                    .withMotorInverted(false)
                    .withClosedLoopRampRate(HoodConstants.CLOSED_LOOP_RAMP_RATE)
                    .withOpenLoopRampRate(HoodConstants.OPEN_LOOP_RAMP_RATE)
                    .withFeedforward(
                            new SimpleMotorFeedforward(
                                    HoodConstants.FEED_FORWARD_KS,
                                    HoodConstants.FEED_FORWARD_KV,
                                    HoodConstants.FEED_FORWARD_KA))
                    .withSimFeedforward(
                            new SimpleMotorFeedforward(
                                    HoodConstants.SIM_FEED_FORWARD_KS,
                                    HoodConstants.SIM_FEED_FORWARD_KV,
                                    HoodConstants.SIM_FEED_FORWARD_KA))
                    .withControlMode(ControlMode.CLOSED_LOOP)
                    .withMotorInverted(true);

    private final SmartMotorController hoodSMC =
            new SparkWrapper(hoodMotor, DCMotor.getNeo550(1), hoodMotorConfig);

    private final PivotConfig hoodConfig =
            new PivotConfig(hoodSMC)
                    .withTelemetry("HoodMechanism", TelemetryVerbosity.HIGH)
                    .withSoftLimits(HoodConstants.BOTTOM_SOFT_LIMIT, HoodConstants.TOP_SOFT_LIMIT)
                    .withHardLimit(Degrees.of(0), Degrees.of(120))
                    .withMOI(HoodConstants.HOOD_LENGTH, HoodConstants.HOOD_WEIGHT)
                    .withStartingPosition(HoodConstants.START_ANGLE);

    private final Pivot hood = new Pivot(hoodConfig);

    private boolean isReady() {
        return hood.getAngle()
                .isNear(
                        hood.getMechanismSetpoint().orElse(Degrees.of(0)),
                        HoodConstants.READY_TOLERANCE);
    }

    public Hood() {}

    public Command set(double dutycycle) {
        return hood.set(dutycycle);
    }

    public final Trigger isReady =
            new Trigger(this::isReady)
                    .debounce(HoodConstants.IS_READY_DELAY, Debouncer.DebounceType.kFalling);

    public void Update() {}

    public Command setAngle(Supplier<Angle> hoodAngle) {
        return hood.setAngle(() -> Degrees.of(90).minus(hoodAngle.get()));
    }

    public Command lowerHood() {
        return setAngle(() -> HoodConstants.BOTTOM_SOFT_LIMIT);
    }

    public Command sysId() {
        return hood.sysId(
                Volts.of(0.5), // maximumVoltage
                Volts.per(Second).of(6), // step
                Seconds.of(8) // duration
                );
    }

    @Override
    public void periodic() {
        hood.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        hood.simIterate();
    }
}
