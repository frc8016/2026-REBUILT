package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {
    private final SparkMax m_turretmotor = new SparkMax(3, MotorType.kBrushless);
    private final SparkMaxConfig m_turretmotorconfig = new SparkMaxConfig();
    private final SparkClosedLoopController m_turretmotorClosedLoopController =
            m_turretmotor.getClosedLoopController();
    private final TargetManager limelightManager;

    public Turret(TargetManager manager) {
        limelightManager = manager;
        m_turretmotorconfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(TurretConstants.P_VALUE, ClosedLoopSlot.kSlot0)
                .i(TurretConstants.I_VALUE, ClosedLoopSlot.kSlot0)
                .d(TurretConstants.D_VALUE, ClosedLoopSlot.kSlot0)
                .outputRange(
                        TurretConstants.OUTPUTRANGE_MIN_VALUE,
                        TurretConstants.OUTPUTRANGE_MAX_VALUE);
        m_turretmotor.configure(
                m_turretmotorconfig,
                com.revrobotics.ResetMode.kResetSafeParameters,
                com.revrobotics.PersistMode.kPersistParameters);
    }

    public void runTurret() {
        double tx = limelightManager.getTX();
        if (!MathUtil.isNear(tx, 0, Constants.TurretConstants.TX_TOLERANCE)) {
            m_turretmotorClosedLoopController.setSetpoint(tx, ControlType.kVelocity);

        } else {
            m_turretmotorClosedLoopController.setSetpoint(0, ControlType.kVelocity);
        }
    }

    public Command Autoaim() {
        return this.runOnce(() -> this.runTurret());
    }
}
