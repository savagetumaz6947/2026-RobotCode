package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooterAngle.shooterAngleSubsystem;
import org.littletonrobotics.junction.Logger;

/** Command to move the shooter angle to a target position and finish when on target. */
public class ShooterAngleAim extends Command {
  private final shooterAngleSubsystem shooterAngle;
  private final double target;

  public ShooterAngleAim(shooterAngleSubsystem shooterAngle, double target) {
    this.shooterAngle = shooterAngle;
    this.target = target;
    addRequirements(shooterAngle);
  }

  @Override
  public void execute() {
    shooterAngle.setTargetLength(target);
    Logger.recordOutput("ShooterAngle/AutoAimTarget", target);
  }

  @Override
  public boolean isFinished() {
    return shooterAngle.atTarget();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      Logger.recordOutput("ShooterAngle/AutoAimInterrupted", true);
    }
    shooterAngle.stop();
  }
}
