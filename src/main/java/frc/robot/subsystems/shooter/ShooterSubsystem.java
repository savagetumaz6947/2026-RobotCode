package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIO.ShooterIOInputs inputs = new ShooterIO.ShooterIOInputs();

  private static ShooterSubsystem system;

  private int readyStableCountHub = 0;
  private int readyStableCountPass = 0;
  private static final int READY_STABLE_CYCLES = 3;

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
    return run(() -> {
          setDutyCycle(dutyCycle);
        })
        .withName("ShooterSpinUp");
  }
  public Command spinUpVelocity(double velocityRotPerSec) {
    return run(() -> {
          setVelocity(velocityRotPerSec);
        })
        .withName("ShooterSpinUpVelocity");
  }
  /**
   * Command to spin up the shooter to hub shooting speed
   *
   * @return A command that spins the shooter to hub speed
   */
  public Command spinUpForHub() {
    return spinUp(ShooterConstants.HUB_DUTY_CYCLE);
  }

  /**
   * Command to spin up the shooter to pass shooting speed
   *
   * @return A command that spins the shooter to pass speed
   */
  public Command spinUpForPass() {
    return spinUp(ShooterConstants.PASS_DUTY_CYCLE);
  }

  /**
   * Command to idle the shooter at a low speed
   *
   * @return A command that idles the shooter
   */
  public Command idle() {
    return run(() -> {
          setDutyCycle(ShooterConstants.IDLE_DUTY_CYCLE);
        })
        .withName("ShooterIdle");
  }

  /**
   * Command to stop the shooter
   *
   * @return A command that stops the shooter
   */
  public Command stopShooter() {
    return runOnce(
            () -> {
              stop();
            })
        .withName("ShooterStop");
  }

  /**
   * Sets the duty cycle of shooter motors
   *
   * @param dutyCycle The target duty cycle (-1.0 to 1.0)
   */
  public void setDutyCycle(double dutyCycle) {
    io.setDutyCycle(dutyCycle);
  }

  public void setVelocity(double velocityRotPerSec) {
    io.setVelocity(velocityRotPerSec);
  }

  public double getVelocity() {
    return inputs.velocityRotPerSec;
  }

  /**
   * Checks if the shooter is ready to shoot (simplified - always true for duty cycle control)
   *
   * @return Always returns true since we're using duty cycle control
   */
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

  /**
   * Gets the {@link ShooterSubsystem} subsystem instance
   *
   * @return The {@link ShooterSubsystem} subsystem instance
   */
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

  /** Gets the {@link ShooterSubsystem} subsystem instance (alias for system()) */
  public static ShooterSubsystem getInstance() {
    return system();
  }
}
