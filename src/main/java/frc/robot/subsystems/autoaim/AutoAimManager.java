package frc.robot.subsystems.autoaim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.subsystems.shooterAngle.shooterAngleSubsystem;
import java.util.function.Supplier;

/**
 * Simple AutoAim manager that computes a shooter-angle target from the robot pose and field aim
 * points and exposes a command to aim the shooter angle.
 */
public class AutoAimManager {
  private final shooterAngleSubsystem shooterAngle;
  private double intercept = 0.02; // mapping intercept (units)
  private double slope = 0.02; // mapping slope (units per meter)
  private double maxLength = 0.2; // maximum actuator travel (units)

  public AutoAimManager(shooterAngleSubsystem shooterAngle) {
    this.shooterAngle = shooterAngle;
  }

  public AutoAimManager(
      shooterAngleSubsystem shooterAngle, double intercept, double slope, double maxLength) {
    this.shooterAngle = shooterAngle;
    this.intercept = intercept;
    this.slope = slope;
    this.maxLength = maxLength;
  }

  /** Compute a target actuator length for aiming at the field hub based on robot pose. */
  public double computeTargetFromPose(Pose2d robotPose) {
    // Choose field aim target based on alliance
    java.util.Optional<Alliance> allianceOpt = DriverStation.getAlliance();
    Alliance alliance = allianceOpt.isPresent() ? allianceOpt.get() : Alliance.Blue;
    Translation3d aim3d =
        (alliance == Alliance.Red)
            ? Constants.FieldPoses.RED_AIM_TARGET
            : Constants.FieldPoses.BLUE_AIM_TARGET;
    // Only use X/Y components for distance
    Pose2d aim2d = new Pose2d(aim3d.getX(), aim3d.getY(), robotPose.getRotation());
    double dx = aim2d.getX() - robotPose.getX();
    double dy = aim2d.getY() - robotPose.getY();
    double dist = Math.hypot(dx, dy);

    double target = intercept + slope * dist;
    // Clamp
    if (target < 0) target = 0;
    if (target > maxLength) target = maxLength;
    return target;
  }

  /** Convenience factory that returns a command which auto-aims using a robot-pose supplier. */
  public edu.wpi.first.wpilibj2.command.Command autoAimAtField(Supplier<Pose2d> robotPoseSupplier) {
    return new frc.robot.commands.AutoAimToField(shooterAngle, robotPoseSupplier, this);
  }
}
