package frc.robot.subsystems.shooterAngle;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;

/**
 * Hardware PWM-based implementation of {@link ShooterAngleIO} for a simple PWM servo connected to a
 * PWM port. This implementation treats the servo as an open-loop position device: commanded target
 * lengths are mapped to servo positions in [0,1]. Estimated position is read from the servo's last
 * set position (WPILib Servo does not provide closed-loop feedback) and scaled to the configured
 * travel length.
 */
public class ShooterAngleIOPwm implements ShooterAngleIO {
  // Physical configuration: travel length and tolerance
  private final double m_length; // total travel length in same units used by callers
  private static final double TOLERANCE = 0.002; // units

  // WPILib Servo (PWM)
  private final Servo servo;

  // Last commanded target (in length units)
  private double commandedTarget = 0.0;

  /**
   * Create a PWM shooter-angle IO on the given PWM channel using the provided travel length.
   *
   * @param pwmChannel PWM channel (e.g. 5)
   * @param travelLength total travel length (units used by callers)
   */
  public ShooterAngleIOPwm(int pwmChannel, double travelLength) {
    this.servo = new Servo(pwmChannel);
    this.m_length = Math.max(0.0, travelLength);
    // initialize servo to 0 position
    this.commandedTarget = 0.0;
    if (m_length > 0.0) {
      this.servo.setPosition(0.0);
    }
  }

  /** Convenience ctor: use PWM channel 5 and a 0.2-unit travel length (matches sim default). */
  public ShooterAngleIOPwm() {
    this(5, 0.2);
  }

  @Override
  public void updateInputs(ShooterAngleIOInputs inputs) {
    double estPos = 0.0;
    if (m_length > 0.0) {
      // WPILib Servo.getPosition returns the last commanded normalized position [0..1]
      estPos = MathUtil.clamp(servo.getPosition(), 0.0, 1.0) * m_length;
    }

    inputs.targetPos = commandedTarget;
    inputs.estPos = estPos;
    inputs.errorLength = commandedTarget - estPos;
    inputs.atTarget = Math.abs(inputs.errorLength) <= TOLERANCE;
  }

  @Override
  public void setTargetLength(double targetLength) {
    // Clamp to physical range [0, m_length]
    double clamped = MathUtil.clamp(targetLength, 0.0, m_length);
    this.commandedTarget = clamped;
    if (m_length > 0.0) {
      double norm = clamped / m_length; // 0..1
      norm = MathUtil.clamp(norm, 0.0, 1.0);
      servo.setPosition(norm);
    }
  }

  @Override
  public void stop() {
    // Hold current position by re-sending the commanded target
    setTargetLength(this.commandedTarget);
  }
}
