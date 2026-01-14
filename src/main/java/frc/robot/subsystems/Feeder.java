package frc.robot.subsystems;

import static frc.robot.Ports.Feeder.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.FaultLogger;

public class Feeder extends SubsystemBase {
  private final TalonFX feederMotor;
  private final TalonFXConfiguration feederConfiguration;

  private final double FEED_VOLTAGE = 3;

  private int kFeederCurrentLimit = 30;

  public Feeder() {
    feederMotor = new TalonFX(FEEDER);
    feederConfiguration = new TalonFXConfiguration();

    feederConfiguration.CurrentLimits.SupplyCurrentLimit = kFeederCurrentLimit;
    feederConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

    feederConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    feederConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    feederConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    feederMotor.getConfigurator().apply(feederConfiguration);

    FaultLogger.register(feederMotor);

    setDefaultCommand(runOnce(this::stopFeeder).andThen(idle()));
  }

  public void stopFeeder() {
    feederMotor.stopMotor();
  }

  public void setVoltageFeeder(double voltage) {
    feederMotor.setVoltage(FEED_VOLTAGE);
  }

  public Command feed() {
    return this.runEnd(() -> setVoltageFeeder(3), this::stopFeeder);
  }
}
