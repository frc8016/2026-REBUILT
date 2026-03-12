package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
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
    private final BallisticsManager ballisticsManager;

    SmartMotorControllerConfig m_turretmotorconfig =
            new SmartMotorControllerConfig(this)
                    .withControlMode(ControlMode.CLOSED_LOOP)
                    .withClosedLoopController(
                            TurretConstants.P_VALUE,
                            TurretConstants.I_VALUE,
                            TurretConstants.D_VALUE,
                            TurretConstants.TRAPAZOIDAL_MAX_VELOCITY,
                            TurretConstants.TRAPAZOIDAL_MAX_ACCELERATION)
                    .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
                    .withIdleMode(MotorMode.BRAKE)
                    .withMotorInverted(false)
                    .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
                    .withStatorCurrentLimit(Amps.of(40))
                    .withClosedLoopRampRate(Seconds.of(0.25))
                    .withOpenLoopRampRate(Seconds.of(0.25))
                    .withSoftLimit(Degrees.of(-180), Degrees.of(180));
    private final SmartMotorController TurrerSMC =
            new SparkWrapper(m_turretmotor, DCMotor.getNeo550(1), m_turretmotorconfig);

    private final PivotConfig TurretConfig =
            new PivotConfig(TurrerSMC)
                    .withTelemetry("TurretMechanism", TelemetryVerbosity.HIGH)
                    .withSoftLimits(
                            TurretConstants.BOTTOM_SOFT_LIMIT, TurretConstants.TOP_SOFT_LIMIT)
                    .withHardLimit(Degrees.of(0), Degrees.of(120))
                    // .withMOI(TurretConstants.TURRET_LENGTH, TurretConstants.TURRET_WEIGHT)
                    .withStartingPosition(TurretConstants.START_ANGLE);

    private final Pivot Turret = new Pivot(TurretConfig);

    public Turret(BallisticsManager ballisticsManager) {
        this.ballisticsManager = ballisticsManager;
    }

    public void runTurret() {
        Angle tx = ballisticsManager.TX().get();
        if (!MathUtil.isNear(tx.magnitude(), 0, Constants.TurretConstants.TX_TOLERANCE)) {
            Turret.setAngle(tx.plus(Turret.getAngle())); // maby working

        } else {
            Turret.setAngle(tx);
        }
    }

    public Command Autoaim() {
        return this.runOnce(() -> this.runTurret());
    }

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
        return turret.setAngle(() -> offset.get().plus(turret.getAngle()));
    }

    @Override
    public void periodic() {
        turret.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        turret.simIterate();
    }
}
