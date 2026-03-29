package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpindexerConstants;

public class Spindexer extends SubsystemBase {

    private final SparkMax spindexer =
            new SparkMax(2, MotorType.kBrushless); // TODO: motor id must be changed

    public Command run() {
        return new StartEndCommand(
                () -> this.spindexer.set(SpindexerConstants.SPINDEXER_SPEED),
                () -> this.spindexer.set(0),
                this);
    }

    public Command reverse() {
        return new StartEndCommand(
                () -> this.spindexer.set(-SpindexerConstants.SPINDEXER_SPEED),
                () -> this.spindexer.set(0),
                this);
    }
}
