package frc.robot.subsystems.climber;

public interface ClimberIO {

  public static class ClimberIOInputs {
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double temperatureCelsius = 0.0;
  }

  /** Update inputs used for logging. */
  public default void updateInputs(ClimberIOInputs inputs) {}

  /**
   * Set duty cycle for climber motors.
   *
   * <p>Convention: the provided dutyCycle is applied to the TOP motor; the BOTTOM motor will
   * receive the negated value. This matches the mechanical gearing where the two motors must run in
   * opposite directions for the climber to move.
   *
   * <p>Range: -1.0 to 1.0
   */
  public default void setDutyCycle(double dutyCycle) {}

  /** Immediately stop climber motors. */
  public default void stop() {}
}
