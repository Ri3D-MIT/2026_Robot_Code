package frc.robot.subsystems;

import static frc.robot.Ports.Shooter.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
// import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.FaultLogger;

public class Shooter extends SubsystemBase {
  private final int kShooterCurrentLimit = 30;

  private final TalonFX topMotor;
  private final TalonFX bottomMotor;

  public Shooter() {
    this.topMotor = new TalonFX(TOP_LEADER);
    this.bottomMotor = new TalonFX(BOTTOM_FOLLOWER);

    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = kShooterCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // TODO gear ratio?
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    topMotor.getConfigurator().apply(config);
    bottomMotor.getConfigurator().apply(config);

    bottomMotor.setControl(new Follower(TOP_LEADER, MotorAlignmentValue.Opposed));

    FaultLogger.register(topMotor);
    FaultLogger.register(bottomMotor);
  }

  public void stopMotors() {
    topMotor.stopMotor();
    bottomMotor.stopMotor();
  }

  public void setShooterVoltage(double voltage) {
    topMotor.setVoltage(voltage);
    // lowerMotor.setVoltage(-voltage);
  }

  public void setVoltageRight(double voltage) {
    // lowerMotor.setVoltage(voltage);
  }

  public Command runShooter(double voltage) {
    return this.runEnd(() -> setShooterVoltage(voltage), () -> stopMotors());
  }

  public Command runVoltageRight(double voltage) {
    return this.runEnd(() -> setVoltageRight(voltage), () -> stopMotors());
  }
}
