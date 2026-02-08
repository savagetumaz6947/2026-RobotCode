package frc.robot.subsystems.shooterAngle;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterAngleIO {
  @AutoLog
  public static class ShooterAngleIOInputs {
    public double targetPos = 0.0;
    public double estPos = 0.0;
    public double errorLength = 0.0;
    public boolean atTarget = false;
  }

  public default void updateInputs(ShooterAngleIOInputs inputs) {}

  public default void setTargetLength(double targetLength) {}

  public default void stop() {}
}
