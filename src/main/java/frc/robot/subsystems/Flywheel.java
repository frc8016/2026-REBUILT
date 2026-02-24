package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FlyWheelConstants;
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

public class Flywheel extends SubsystemBase {

    private final SparkMax flywheelMotor = new SparkMax(4, MotorType.kBrushless);
    private final SparkMax flywheelMotorfollower = new SparkMax(5, MotorType.kBrushless);
    private final Distance flywheelDiameter = Inches.of(4);

    private final SmartMotorControllerConfig motorConfig =
            new SmartMotorControllerConfig(this)
                    .withClosedLoopController(
                            FlyWheelConstants.PROPORTIONAL,
                            FlyWheelConstants.INTEGRAL,
                            FlyWheelConstants.DERIVATIVE,
                            FlyWheelConstants.TRAPAZOIDAL_MAX_VELOCITY,
                            FlyWheelConstants.TRAPAZOIDAL_MAX_ACCELERATION)
                    .withGearing(new MechanismGearing(GearBox.fromReductionStages(1, 1)))
                    .withIdleMode(MotorMode.COAST)
                    .withTelemetry("FlywheelMotor", TelemetryVerbosity.HIGH)
                    .withStatorCurrentLimit(FlyWheelConstants.STATOR_CURRENT_LIMIT)
                    .withMotorInverted(false)
                    .withClosedLoopRampRate(FlyWheelConstants.CLOSED_LOOP_RAMP_RATE)
                    .withOpenLoopRampRate(FlyWheelConstants.OPEN_LOOP_RAMP_RATE)
                    .withFeedforward(
                            new SimpleMotorFeedforward(
                                    FlyWheelConstants.FEED_FORWARD_KS,
                                    FlyWheelConstants.FEED_FORWARD_KV,
                                    FlyWheelConstants.FEED_FORWARD_KA))
                    .withSimFeedforward(
                            new SimpleMotorFeedforward(
                                    FlyWheelConstants.SIM_FEED_FORWARD_KS,
                                    FlyWheelConstants.SIM_FEED_FORWARD_KV,
                                    FlyWheelConstants.SIM_FEED_FORWARD_KA))
                    .withControlMode(ControlMode.CLOSED_LOOP)
                    .withFollowers(Pair.of(flywheelMotorfollower, true));

    private final SmartMotorController motor =
            new SparkWrapper(flywheelMotor, DCMotor.getNEO(1), motorConfig);

    private final FlyWheelConfig flywheelConfig =
            new FlyWheelConfig(motor)
                    .withDiameter(FlyWheelConstants.FLYWHEEL_DIAMETER)
                    .withMass(FlyWheelConstants.FLYWHEEL_MASS)
                    .withTelemetry("FlywheelMech", TelemetryVerbosity.HIGH)
                    .withSoftLimit(
                            FlyWheelConstants.SOFT_LIMIT.negate(), FlyWheelConstants.SOFT_LIMIT);
    // .withSpeedometerSimulation(RPM.of(7500)); // optional to make graph of velocity not position

    private final FlyWheel flywheel = new FlyWheel(flywheelConfig);

    private boolean isReady() {
        return MathUtil.isNear(
                FlyWheelConstants.SHOOTING_SETPOINT.in(MetersPerSecond),
                flywheel.getSpeed().in(RotationsPerSecond)
                        * flywheelDiameter.times(Math.PI).in(Meters),
                FlyWheelConstants.READY_TOLERANCE);
    }

    public Flywheel() {}

    public Command spinFlywheel() {
        System.out.println("SHOOT");
        return this.flywheel.setSpeed(
                RotationsPerSecond.of(
                        FlyWheelConstants.SHOOTING_SETPOINT.in(MetersPerSecond)
                                / flywheelDiameter.times(Math.PI).in(Meters)));
    }

    public Command idleFlywheel() {
        System.out.println("IDLE");
        return this.flywheel.setSpeed(
                RotationsPerSecond.of(
                        FlyWheelConstants.IDLE_SETPOINT.in(MetersPerSecond)
                                / flywheelDiameter.times(Math.PI).in(Meters)));
    }

    public final Trigger isReady =
            new Trigger(this::isReady)
                    // Stay ready for short time after to prevent flapping
                    .debounce(FlyWheelConstants.IS_READY_DELAY, Debouncer.DebounceType.kFalling);

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
