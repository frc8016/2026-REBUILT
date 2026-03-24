package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.TurretConstants;
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

public class Turret extends SubsystemBase {
    private final SparkMax m_turretmotor = new SparkMax(3, MotorType.kBrushless);

    SmartMotorControllerConfig m_turretmotorconfig =
            new SmartMotorControllerConfig(this)
                    .withControlMode(ControlMode.CLOSED_LOOP)
                    .withClosedLoopController(
                            TurretConstants.P_VALUE,
                            TurretConstants.I_VALUE,
                            TurretConstants.D_VALUE)
                    .withGearing(new MechanismGearing(GearBox.fromReductionStages(40)))
                    .withExponentialProfile(
                            Volts.of(12),
                            TurretConstants.MAX_VELOCITY,
                            TurretConstants.MAX_ACCELERATION)
                    .withIdleMode(MotorMode.BRAKE)
                    .withMotorInverted(false)
                    .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
                    .withStatorCurrentLimit(TurretConstants.STATOR_CURRENT_LIMIT)
                    .withClosedLoopRampRate(TurretConstants.CLOSED_LOOP_RAMP_RATE)
                    .withOpenLoopRampRate(TurretConstants.OPEN_LOOP_RAMP_RATE);
    private final SmartMotorController TurrerSMC =
            new SparkWrapper(m_turretmotor, DCMotor.getNeo550(1), m_turretmotorconfig);

    private final PivotConfig TurretConfig =
            new PivotConfig(TurrerSMC)
                    .withTelemetry("TurretMechanism", TelemetryVerbosity.HIGH)
                    .withSoftLimits(
                            TurretConstants.BOTTOM_SOFT_LIMIT, TurretConstants.TOP_SOFT_LIMIT)
                    .withHardLimit(Degrees.of(0), Degrees.of(220))
                    .withMOI(TurretConstants.TURRET_LENGTH, TurretConstants.TURRET_WEIGHT)
                    .withStartingPosition(TurretConstants.START_ANGLE);

    public Turret() {}

    private final Pivot turret = new Pivot(TurretConfig);

    private boolean isReady() {
        return turret.getAngle()
                .isNear(
                        turret.getMechanismSetpoint().orElse(Degrees.of(0)),
                        TurretConstants.READY_TOLERANCE);
    }

    public final Trigger isReady =
            new Trigger(this::isReady)
                    .debounce(TurretConstants.IS_READY_DELAY, Debouncer.DebounceType.kFalling);

    public Command setAngle(Supplier<Angle> offset) {
        return turret.setAngle(() -> turret.getAngle().minus(offset.get()));
    }

    public Angle getAngle() {
        return turret.getAngle();
    }

    public Command sysId() {
        return turret.sysId(Volts.of(1), Volts.of(7).per(Second), Seconds.of(5));
    }

    @Override
    public void periodic() {
        turret.updateTelemetry();
        SmartDashboard.putBoolean("turretIsReady", isReady());
    }

    @Override
    public void simulationPeriodic() {
        turret.simIterate();
    }
}
