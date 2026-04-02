package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeRoller extends SubsystemBase {
    private final SparkMax intake =
            new SparkMax(8, MotorType.kBrushless); // Might need to change motor id

    public Command spinForwards() {
        return new StartEndCommand(() -> this.intake.set(-1), () -> this.intake.set(0), this);
    }

    public Command spinBackwards() {
        return new StartEndCommand(() -> this.intake.set(0.5), () -> this.intake.set(0), this);
    }
}
