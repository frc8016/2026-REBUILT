package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.LimelightHelpers;

public class Turret {
  double Kp = -0.1;
  double min_command = 0.05;
  // program motor
  private final LimelightHelpers LimelightHelpers = new LimelightHelpers();

  public Command aimShooter() {
    return Commands.runOnce(() -> {});
  }

  // private Command Turn
  private void trunMotors() {
    while (true) {}
  }

  // set a while loop outside of command that checks up on lime data( treshhold are we aimed or not
  // ) - with that data aim motor

}
