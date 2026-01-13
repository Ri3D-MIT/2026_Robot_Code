// intake skeleton code
package frc.robot.subsystems;

import static frc.robot.Ports.Intake.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.FaultLogger;

public class Intake extends SubsystemBase {
  // Current Limits
  private final int kIntakeCurrentLimit = 15;
  private final int kPivotCurrentLimit = 10;

  // Limit switch channels
  private int kPivotLimitSwitchChannel = 0;

  // TODO: Determine and set constraints for intake motors and pivot
  // Configure PID for intake motors
  private final double kP_intake = 0;
  private final double kD_intake = 0;

  private final PIDController pidIntake = new PIDController(kP_intake, 0, kD_intake) ;


  // Feedforward mechanism for intake pivot
  private final double kS_pivot = 0;
  private final double kV_pivot = 0;
  private final double kA_pivot = 0;

  private final SimpleMotorFeedforward ffPivot = new SimpleMotorFeedforward(kS_pivot, kV_pivot,  kA_pivot);

  // Configure PID for Intake pivot
  private final double kP_pivot = 0;
  private final double kD_pivot = 0;

  private final PIDController pidPivot = new PIDController(kP_intake, 0, kD_intake);

  // PID tolerances for intake and pivot
  private final double kToleranceIntake = 10;
  private final double kTolerancePivot = 10;

  // Kraken X60 that controls intake mechanism.
  private TalonFX intakeMotor;
  private TalonFXConfiguration intakeConfig;

  // Kraken X60 controls adjustment mechanism. Compatible with CANivore.
  private TalonFX pivotMotor;
  private TalonFXConfiguration pivotConfig;

  // Create limit switch object
  private DigitalInput pivotLimitSwitch;

  // Booleans
  // TODO: if two limit switches are to be installed, integrate minimium pivot code
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
    this.intakeMotor = new TalonFX(INTAKE_MOTOR);
    this.intakeConfig = new TalonFXConfiguration();

    // Set current limit for the intake TalonFX motor controller
    intakeConfig.CurrentLimits.SupplyCurrentLimit = kIntakeCurrentLimit;
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    // Create TalonFX object, configurations
    this.pivotMotor = new TalonFX(PIVOT_MOTOR);
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


    // Fault Logger registering
    FaultLogger.register(intakeMotor);
    FaultLogger.register(pivotMotor);

    pidIntake.setTolerance(kToleranceIntake);
    pidPivot.setTolerance(kToleranceIntake);
  }

  // Stop both motors
  public void stop() {
    intakeMotor.stopMotor();
    pivotMotor.stopMotor();
  }

  // Intake pivot
  public void setVoltagePivot(double voltage) {
    
    // stupid comparison logic. TODO: please simplify :( 

    // If a limit switch is tripped on said direction, robot should not be able to pivot further than that point
    if ((voltage < 0 && reachedMinPivot) || (voltage > 0 && reachedMaxPivot)) {
      voltage = 0;
    }

    // If a robot moves the opposite direction, reset the limit booleans.
    if ((voltage > 0 && reachedMinPivot)||(voltage<0&&reachedMaxPivot)) {
      reachedMaxPivot = false;
      reachedMinPivot = false;
    }
        
    this.pivotMotor.setVoltage(voltage);

    // When limit switch triggered, prevent additional input on that direction only. 
    if (pivotLimitSwitch.get() == true) {
      reachedMaxPivot = true;
    }
    
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