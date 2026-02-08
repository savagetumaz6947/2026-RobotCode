package frc.robot.subsystems.shooterAngle;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

/**
 * Simulation implementation of {@link ShooterAngleIO} that mimics the behavior of a linear
 * servo/actuator using a simple velocity-limited model. This mirrors the provided LinearServo
 * pattern: it computes a time delta each update and moves the simulated position toward the
 * commanded target at up to the configured max speed.
 */
public class ShooterAngleIOSim implements ShooterAngleIO {
  // Simulated configuration (defaults — can be tuned)
  private double m_length = 0.2; // units (meters or subsystem-specific units)
  private double m_speed = 0.05; // units per second (max speed)

  // Internal state
  private double setPos = 0.0; // commanded target
  private double curPos = 0.0; // current estimated position
  private double lastTime = Timer.getFPGATimestamp();

  // Tolerance for considering at target
  private static final double TOLERANCE = 0.002; // units

  public ShooterAngleIOSim() {}

  /**
   * Construct a sim with explicit travel length and maximum speed.
   *
   * @param length total travel length (same units used by set/get)
   * @param maxSpeed maximum speed in units/second
   */
  public ShooterAngleIOSim(double length, double maxSpeed) {
    this.m_length = Math.max(0.0, length);
    this.m_speed = Math.max(0.0, maxSpeed);
    this.setPos = 0.0;
    this.curPos = 0.0;
    this.lastTime = Timer.getFPGATimestamp();
  }

  @Override
  public void updateInputs(ShooterAngleIOInputs inputs) {
    double now = Timer.getFPGATimestamp();
    double dt = now - lastTime;
    if (dt <= 0) {
      dt = 0.02; // fallback to 20ms
    }

    // Move curPos toward setPos limited by max speed
    double maxDelta = m_speed * dt;
    if (curPos > setPos + maxDelta) {
      curPos -= maxDelta;
    } else if (curPos < setPos - maxDelta) {
      curPos += maxDelta;
    } else {
      curPos = setPos;
    }

    // Fill inputs
    inputs.targetPos = setPos;
    inputs.estPos = curPos;
    inputs.errorLength = setPos - curPos;
    inputs.atTarget = Math.abs(inputs.errorLength) <= TOLERANCE;

    lastTime = now;
  }

  @Override
  public void setTargetLength(double targetLength) {
    // Clamp to physical range [0, m_length]
    this.setPos = MathUtil.clamp(targetLength, 0.0, m_length);
  }

  @Override
  public void stop() {
    // Hold current position
    this.setPos = this.curPos;
  }

  /** Convenience: set target position in same units as the sim (alias) */
  public void setPosition(double target) {
    setTargetLength(target);
  }

  /**
   * Convenience: update the internal position using the same logic as updateInputs. Call this if
   * you need to advance the sim outside of the normal updateInputs path.
   */
  public void updateCurPos() {
    double now = Timer.getFPGATimestamp();
    double dt = now - lastTime;
    if (dt <= 0) {
      dt = 0.02;
    }

    double maxDelta = m_speed * dt;
    if (curPos > setPos + maxDelta) {
      curPos -= maxDelta;
    } else if (curPos < setPos - maxDelta) {
      curPos += maxDelta;
    } else {
      curPos = setPos;
    }

    lastTime = now;
  }

  /** Get current estimated position (same units as setTargetLength) */
  public double getPosition() {
    return curPos;
  }

  /** Returns true if the sim is at the commanded target. */
  public boolean isFinished() {
    return Math.abs(setPos - curPos) <= TOLERANCE;
  }
}
