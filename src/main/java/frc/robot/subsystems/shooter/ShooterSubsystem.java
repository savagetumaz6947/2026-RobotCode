package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class ShooterSubsystem extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIO.ShooterIOInputs inputs = new ShooterIO.ShooterIOInputs();

  private static ShooterSubsystem system;
  // Readiness debounce counters to avoid transient oscillation preventing conveyor start
  private int readyStableCountHub = 0;
  private int readyStableCountPass = 0;
  private static final int READY_STABLE_CYCLES = 3; // reduce debounce so feeder starts faster

  /** Constructs a {@link ShooterSubsystem} subsystem instance */
  private ShooterSubsystem(ShooterIO io) {
    this.io = io;
  }

  /**
   * Command to spin up the shooter to a specific duty cycle
   *
   * @param dutyCycle The target duty cycle (-1.0 to 1.0)
   * @return A command that spins the shooter to the target duty cycle
   */
  public Command spinUp(double dutyCycle) {
    return run(() -> setDutyCycle(dutyCycle)).withName("ShooterSpinUp");
  }

  /** Command to spin up the shooter to a specific velocity (rotations per second) */
  public Command spinUpVelocity(double velocityRps) {
    return run(() -> setVelocity(velocityRps)).withName("ShooterSpinUpVelocity");
  }

  /** Command to spin up the shooter to hub shooting speed */
  public Command spinUpForHub() {
    // Prefer velocity-based spin up; keep duty-cycle backward compatibility removed
    return spinUpVelocity(ShooterConstants.HUB_VELOCITY_RPS);
  }

  /** Command to spin up the shooter to pass shooting speed */
  public Command spinUpForPass() {
    return spinUpVelocity(ShooterConstants.PASS_VELOCITY_RPS);
  }

  /**
   * Spin up shooter based on a measured distance to the hub. This uses a simple linear
   * interpolation between PASS_VELOCITY_RPS (close) and HUB_VELOCITY_RPS (far) over the
   * DISTANCE_MIN_METERS..DISTANCE_MAX_METERS range. Values outside the range are clamped.
   *
   * @param distanceMeters Distance from the camera to the hub in meters.
   * @return A command which applies the computed velocity while scheduled.
   */
  public Command spinUpForDistance(double distanceMeters) {
    return run(() -> setVelocityForDistance(distanceMeters)).withName("ShooterSpinUpForDistance");
  }

  /** Command to idle the shooter at a low speed */
  public Command idle() {
    return run(() -> setDutyCycle(ShooterConstants.IDLE_DUTY_CYCLE)).withName("ShooterIdle");
  }

  /** Command to stop the shooter */
  public Command stopShooter() {
    return runOnce(this::stop).withName("ShooterStop");
  }

  /** Sets the duty cycle of the leader motor (follower mirrors automatically) */
  public void setDutyCycle(double dutyCycle) {
    io.setDutyCycle(dutyCycle);
  }

  /** Sets a velocity target for the shooter (rotations per second). */
  public void setVelocity(double velocityRotPerSec) {
    io.setVelocity(velocityRotPerSec);
  }

  /**
   * Compute a velocity target from a distance measurement and apply it to the shooter. Linear
   * interpolation between PASS_VELOCITY_RPS (at DISTANCE_MIN_METERS) and HUB_VELOCITY_RPS (at
   * DISTANCE_MAX_METERS).
   *
   * @param distanceMeters distance from camera to target (meters)
   */
  public void setVelocityForDistance(double distanceMeters) {
    double minD = ShooterConstants.DISTANCE_MIN_METERS;
    double maxD = ShooterConstants.DISTANCE_MAX_METERS;

    // avoid division by zero
    double t = 0.0;
    if (maxD > minD) {
      t = (distanceMeters - minD) / (maxD - minD);
    }
    // clamp 0..1
    t = Math.max(0.0, Math.min(1.0, t));

    double minV = ShooterConstants.PASS_VELOCITY_RPS;
    double maxV = ShooterConstants.HUB_VELOCITY_RPS;
    double targetV = minV + t * (maxV - minV);

    // Apply computed velocity
    setVelocity(targetV);
    Logger.recordOutput("Shooter/TargetVelocityFromDistance", targetV);
    Logger.recordOutput("Shooter/DistanceMeters", distanceMeters);
  }

  /** Gets the shooter motor velocity */
  public double getVelocity() {
    return inputs.velocityRotPerSec;
  }

  /** Checks if the shooter is ready to shoot (simplified for duty cycle) */
  public boolean readyForHub() {
    return readyStableCountHub >= READY_STABLE_CYCLES;
  }

  public boolean readyForPass() {
    return readyStableCountPass >= READY_STABLE_CYCLES;
  }

  /** Stops the shooter motors */
  public void stop() {
    io.stop();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.recordOutput("Shooter/VelocityRotPerSec", inputs.velocityRotPerSec);
    Logger.recordOutput("Shooter/CurrentAmps", inputs.currentAmps);

    // Update readiness counters (debounce)
    if (Math.abs(inputs.velocityRotPerSec - ShooterConstants.HUB_VELOCITY_RPS)
        <= ShooterConstants.VELOCITY_TOLERANCE_RPS) {
      readyStableCountHub = Math.min(READY_STABLE_CYCLES, readyStableCountHub + 1);
    } else {
      readyStableCountHub = 0;
    }

    if (Math.abs(inputs.velocityRotPerSec - ShooterConstants.PASS_VELOCITY_RPS)
        <= ShooterConstants.VELOCITY_TOLERANCE_RPS) {
      readyStableCountPass = Math.min(READY_STABLE_CYCLES, readyStableCountPass + 1);
    } else {
      readyStableCountPass = 0;
    }

    Logger.recordOutput("Shooter/ReadyStableCountHub", readyStableCountHub);
    Logger.recordOutput("Shooter/ReadyStableCountPass", readyStableCountPass);
  }

  /** Gets the {@link ShooterSubsystem} singleton */
  public static ShooterSubsystem system() {
    if (system == null) {
      if (RobotBase.isSimulation()) {
        system = new ShooterSubsystem(new ShooterIOSim());
      } else {
        system = new ShooterSubsystem(new ShooterIOTalonFX());
      }
    }
    return system;
  }

  /** Alias for system() */
  public static ShooterSubsystem getInstance() {
    return system();
  }
}
