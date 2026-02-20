// intake skeleton code
package frc.robot.subsystems;

import static frc.robot.Ports.Intake.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.FaultLogger;

public class Intake extends SubsystemBase implements AutoCloseable {
  // Current Limits
  private final int kIntakeCurrentLimit = 15;
  private final int kPivotCurrentLimit = 10;

  // TODO MUST FIX VALUE
  private static final double MAX_PIVOT_POS = 75;

  // Kraken X60 that controls intake mechanism.
  private TalonFX intakeMotor;
  private TalonFX pivotMotor;

  public Intake() {
    // Create TalonFX object and configuration for intake motor
    this.intakeMotor = new TalonFX(INTAKE_MOTOR);
    var intakeConfig = new TalonFXConfiguration();

    // Set current limit for the intake TalonFX motor controller
    intakeConfig.CurrentLimits.SupplyCurrentLimit = kIntakeCurrentLimit;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    intakeMotor.getConfigurator().apply(intakeConfig);

    FaultLogger.register(intakeMotor);

    // Create TalonFX object, configurations
    this.pivotMotor = new TalonFX(PIVOT_MOTOR);
    var pivotConfig = new TalonFXConfiguration();

    // Set current limit for the pivot TalonFX motor controller
    pivotConfig.CurrentLimits.SupplyCurrentLimit = kPivotCurrentLimit;
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Set neutral mode and sensor source for Talon FX
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    // TODO add gear ratio

    // Apply configurations to TalonFX
    pivotMotor.getConfigurator().apply(pivotConfig);

    FaultLogger.register(pivotMotor);

    setDefaultCommand(stopPivot().asProxy().alongWith(stopIntake()));

    new Trigger(() -> pivotPosition() >= MAX_PIVOT_POS || pivotPosition() <= 0).onTrue(stopPivot());
  }

  public Command stopIntake() {
    return runOnce(intakeMotor::stopMotor).andThen(idle());
  }

  public Command stopPivot() {
    return runOnce(pivotMotor::stopMotor).andThen(idle());
  }

  public void setIntakeVoltage(double voltage) {
    intakeMotor.setVoltage(voltage);
  }

  public void setPivotVoltage(double voltage) {
    pivotMotor.setVoltage(voltage);
  }

  public double intakeVelocity() {
    return intakeMotor.getVelocity().getValueAsDouble();
  }

  @Logged
  public double pivotPosition() {
    return pivotMotor.getPosition().getValueAsDouble();
  }

  public Command ground() {
    return Commands.run(() -> setPivotVoltage(0.5)).alongWith(stopIntake());
  }

  // TODO negative...?
  public Command stow() {
    return Commands.run(() -> setPivotVoltage(-0.5)).alongWith(stopIntake());
  }

  @Override
  public void close() throws Exception {
    intakeMotor.close();
    pivotMotor.close();
  }
}
