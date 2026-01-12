// intake skeleton code

package frc.robot.subsystems;

/*
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
*/

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  // TalonFX Motor IDs
  private final int kIntakeID = 35;
  private final int kPivotID = 36;

  // Current Limits
  private final int kIntakeCurrentLimit = 15;
  private final int kPivotCurrentLimit = 10;

  // Limit switch channels
  private int kPivotLimitSwitchChannel = 0;

  // Kraken X60 that controls intake mechanism.
  private TalonFX intakeMotor;
  private TalonFXConfiguration intakeConfig;

  // Kraken X60 controls adjustment mechanism. Compatible with CANivore.
  private TalonFX pivotMotor;
  private TalonFXConfiguration pivotConfig;

  // Create limit switch object
  private DigitalInput pivotLimitSwitch;

  // Booleans
  private boolean reachedMinPivot = false;
  private boolean reachedMaxPivot = false;

  /* No longer using SparkMAX.
  // SparkMAX configuration
  // SparkMAX controls NEO 550. NOT compatible with CANivore.
  private SparkMax intakeMotor;
  private SparkMaxConfig intakeConfig;
  */

  public Intake() {
    /* No longer using SparkMAX

    // Create SparkMAX object
    this.intakeMotor = new SparkMax(kIntakeID, MotorType.kBrushless);

    // Create SparkMAX Configs object, set current limit and idle mode
    this.intakeConfig = new SparkMaxConfig();
    intakeConfig
        .smartCurrentLimit(kIntakeCurrentLimit)
        .idleMode(IdleMode.kBrake);

    // Apply SparkMAX configurations: can choose to reset safe parameters or save parameters to controller
    intakeMotor.configure(intakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    */

    // Create TalonFX object and configuration for intake motor
    this.intakeMotor = new TalonFX(kIntakeID);
    this.intakeConfig = new TalonFXConfiguration();

    // Set current limit for the intake TalonFX motor controller
    intakeConfig.CurrentLimits.SupplyCurrentLimit = kIntakeCurrentLimit;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    // Create TalonFX object, configurations
    this.pivotMotor = new TalonFX(kPivotID);
    this.pivotConfig = new TalonFXConfiguration();

    // Set current limit for the pivot TalonFX motor controller
    pivotConfig.CurrentLimits.SupplyCurrentLimit = kPivotCurrentLimit;
    pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    // Set neutral mode and sensor source for Talon FX
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    // Apply configurations to TalonFX
    pivotMotor.getConfigurator().apply(pivotConfig);

    // Create limit switch reference
    pivotLimitSwitch = new DigitalInput(kPivotLimitSwitchChannel);
  }

  // Stop both motors
  public void stop() {
    intakeMotor.stopMotor();
    pivotMotor.stopMotor();
  }

  // Intake pivot
  public void setVoltagePivot(double voltage) {
    this.pivotMotor.setVoltage(voltage);
  }

  public Command adjustIntakePivot(double voltage) {
    return this.runEnd(() -> setVoltagePivot(voltage), () -> stop());
  }

  // END intake pivot

  // Fuel Intake
  public void setVoltageFuelIntake(double voltage) {
    intakeMotor.setVoltage(voltage);
  }

  public Command intakeFuelPiece(double voltage) {
    return this.runEnd(() -> setVoltageFuelIntake(voltage), () -> stop());
  }
  // END fuel intake

}
