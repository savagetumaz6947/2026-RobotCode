package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {

  private final ClimberIO io;
  private final ClimberIO.ClimberIOInputs inputs = new ClimberIO.ClimberIOInputs();

  private static ClimberSubsystem instance;

  private ClimberSubsystem(ClimberIO io) {
    this.io = io;
  }

  public static ClimberSubsystem getInstance() {
    if (instance == null) {
      if (RobotBase.isReal()) {
        instance = new ClimberSubsystem(new ClimberIOTalonFX());
      } else {
        instance = new ClimberSubsystem(new ClimberIOTalonFX());
      }
    }
    return instance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.recordOutput("Climber/AppliedVolts", inputs.appliedVolts);
    Logger.recordOutput("Climber/CurrentAmps", inputs.currentAmps);
    Logger.recordOutput("Climber/TemperatureCelsius", inputs.temperatureCelsius);
  }

  /** Command to run climber at configured duty while held. */
  public Command climbWhileHeld() {
    return climbWhileHeld(ClimberConstants.CLIMB_DUTY);
  }

  /**
   * Command to run climber at a specific duty while held. Positive duty applies to top motor and
   * negative to bottom motor (bottom gets negated inside IO). Use a negative duty to run the
   * mechanism in reverse.
   */
  public Command climbWhileHeld(double duty) {
    return run(() -> io.setDutyCycle(duty)).withName("ClimbWhileHeld");
  }

  /** Command to stop climber immediately. */
  public Command stop() {
    return runOnce(() -> io.stop()).withName("ClimberStop");
  }

  /** Stop motors immediately. */
  public void stopMotor() {
    io.stop();
  }
}
