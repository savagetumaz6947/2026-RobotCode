package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.autoaim.AutoAimManager;
import frc.robot.subsystems.shooterAngle.shooterAngleSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/** Command that computes a target from robot pose and drives the shooterAngleSubsystem to it. */
public class AutoAimToField extends Command {
  private final shooterAngleSubsystem shooterAngle;
  private final Supplier<Pose2d> robotPoseSupplier;
  private final AutoAimManager manager;

  public AutoAimToField(
      shooterAngleSubsystem shooterAngle,
      Supplier<Pose2d> robotPoseSupplier,
      AutoAimManager manager) {
    this.shooterAngle = shooterAngle;
    this.robotPoseSupplier = robotPoseSupplier;
    this.manager = manager;
    addRequirements(shooterAngle);
  }

  @Override
  public void execute() {
    Pose2d pose = robotPoseSupplier.get();
    if (pose == null) {
      return;
    }
    double target = manager.computeTargetFromPose(pose);
    shooterAngle.setTargetLength(target);
    Logger.recordOutput("AutoAim/ComputedTarget", target);
  }

  @Override
  public boolean isFinished() {
    return shooterAngle.atTarget();
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      Logger.recordOutput("AutoAim/Interrupted", true);
    }
    shooterAngle.stop();
  }
}
