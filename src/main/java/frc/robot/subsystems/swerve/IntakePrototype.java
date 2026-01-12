// intake skeleton code

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    // Adjust motor IDs
    private int kIntakeID = 35;
    private int kPivotID = 36;

    // SparkMAX configuration
    // SparkMAX controls NEO 550. NOT compatible with CANivore.
    private SparkMax intakeMotor;
    private SparkMaxConfig intakeConfig;
    private int kIntakeCurrentLimit = 30;

    // Kraken X60 controls adjustment mechanism. Compatible with CANivore.
    private TalonFX pivotMotor;
    private TalonFXConfiguration pivotConfig;
    private int kPivotCurrentLimit = 30;

    public Intake() {
        // Create SparkMAX object
        this.intakeMotor = new SparkMax(kIntakeID, MotorType.kBrushless);

        // Create SparkMAX Configs object, set current limit and idle mode
        this.intakeConfig = new SparkMaxConfig();
        intakeConfig
            .smartCurrentLimit(kIntakeCurrentLimit)
            .idleMode(IdleMode.kBrake);
        
        // Apply SparkMAX configurations: can choose to reset safe parameters or save parameters to controller
        intakeMotor.configure(intakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // Create TalonFX object, configurations
        this.pivotMotor = new TalonFX(kPivotID);
        this.pivotConfig = new TalonFXConfiguration();

        // Set current limit for TalonFX
        pivotConfig.CurrentLimits.SupplyCurrentLimit = kPivotCurrentLimit;
        pivotConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        // Set neutral mode and sensor source for Talon FX
        pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        // Apply configurations to TalonFX
        pivotMotor.getConfigurator().apply(pivotConfig);
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
