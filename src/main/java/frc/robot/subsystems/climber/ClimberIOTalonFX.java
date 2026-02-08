package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.Constants.ClimberConstants;

public class ClimberIOTalonFX implements ClimberIO {

  private final TalonFX topMotor;
  private final TalonFX bottomMotor;
  private final DutyCycleOut dutyControl = new DutyCycleOut(0);

  public ClimberIOTalonFX() {
    topMotor = new TalonFX(ClimberConstants.TOP_MOTOR_CAN_ID, ClimberConstants.CAN_BUS);
    bottomMotor = new TalonFX(ClimberConstants.BOTTOM_MOTOR_CAN_ID, ClimberConstants.CAN_BUS);

    TalonFXConfiguration topConfig = new TalonFXConfiguration();
    topConfig.CurrentLimits.StatorCurrentLimit = ClimberConstants.STATOR_CURRENT_LIMIT;
    topConfig.CurrentLimits.SupplyCurrentLimit = ClimberConstants.SUPPLY_CURRENT_LIMIT;
    topConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    topConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    topConfig.MotorOutput.Inverted =
        ClimberConstants.TOP_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    topConfig.MotorOutput.NeutralMode = ClimberConstants.NEUTRAL_MODE;

    TalonFXConfiguration bottomConfig = new TalonFXConfiguration();
    bottomConfig.CurrentLimits.StatorCurrentLimit = ClimberConstants.STATOR_CURRENT_LIMIT;
    bottomConfig.CurrentLimits.SupplyCurrentLimit = ClimberConstants.SUPPLY_CURRENT_LIMIT;
    bottomConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    bottomConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    bottomConfig.MotorOutput.Inverted =
        ClimberConstants.BOTTOM_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    bottomConfig.MotorOutput.NeutralMode = ClimberConstants.NEUTRAL_MODE;

    topMotor.getConfigurator().apply(topConfig);
    bottomMotor.getConfigurator().apply(bottomConfig);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.appliedVolts = topMotor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps =
        (topMotor.getStatorCurrent().getValueAsDouble()
                + bottomMotor.getStatorCurrent().getValueAsDouble())
            / 2.0;
    inputs.temperatureCelsius =
        Math.max(
            topMotor.getDeviceTemp().getValueAsDouble(),
            bottomMotor.getDeviceTemp().getValueAsDouble());
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    // Apply duty to top motor and negated duty to bottom motor per mechanical coupling
    // (top = +0.2, bottom = -0.2 when dutyCycle = 0.2).
    topMotor.setControl(dutyControl.withOutput(dutyCycle));
    bottomMotor.setControl(dutyControl.withOutput(-dutyCycle));
  }

  @Override
  public void stop() {
    topMotor.stopMotor();
    bottomMotor.stopMotor();
  }
}
