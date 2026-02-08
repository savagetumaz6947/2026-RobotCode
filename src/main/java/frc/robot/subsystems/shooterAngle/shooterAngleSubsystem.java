package frc.robot.subsystems.shooterAngle;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** High-level subsystem for the shooter angle mechanism. */
public class shooterAngleSubsystem extends SubsystemBase {
  private final ShooterAngleIO io;
  private final ShooterAngleIO.ShooterAngleIOInputs inputs =
      new ShooterAngleIO.ShooterAngleIOInputs();

  private static shooterAngleSubsystem system;

  private shooterAngleSubsystem(ShooterAngleIO io) {
    this.io = io;
  }

  /** Command that moves the shooter angle to a target position (repeatedly commands target). */
  public Command moveTo(double target) {
    return run(() -> setTargetLength(target)).withName("ShooterAngleMoveTo");
  }

  /**
   * Convenience: returns a command that auto-aims to the provided target and finishes when reached.
   */
  public Command autoAimTo(double target) {
    return new frc.robot.commands.AutoAim(this, target).withName("AutoAim");
  }

  /** Command that stops the shooter angle motion (holds current position). */
  public Command stopCommand() {
    return runOnce(this::stop).withName("ShooterAngleStop");
  }

  public void setTargetLength(double targetLength) {
    io.setTargetLength(targetLength);
  }

  public void stop() {
    io.stop();
  }

  public double getEstimatedPosition() {
    return inputs.estPos;
  }

  public double getTargetPosition() {
    return inputs.targetPos;
  }

  public boolean atTarget() {
    return inputs.atTarget;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.recordOutput("ShooterAngle/TargetPos", inputs.targetPos);
    Logger.recordOutput("ShooterAngle/EstPos", inputs.estPos);
    Logger.recordOutput("ShooterAngle/ErrorLength", inputs.errorLength);
    Logger.recordOutput("ShooterAngle/AtTarget", inputs.atTarget);
  }

  /** Singleton accessor. Uses sim implementation when running in simulation. */
  public static shooterAngleSubsystem system() {
    if (system == null) {
      if (RobotBase.isSimulation()) {
        system = new shooterAngleSubsystem(new ShooterAngleIOSim());
      } else {
        // Use PWM-based hardware IO on real robot (servo on PWM channel 5).
        // If you have a different PWM channel or travel length, update here or
        // provide a config constant.
        system = new shooterAngleSubsystem(new ShooterAngleIOPwm(5, 0.2));
      }
    }
    return system;
  }

  public static shooterAngleSubsystem getInstance() {
    return system();
  }
}
