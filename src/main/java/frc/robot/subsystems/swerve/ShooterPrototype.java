package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private final int kTopMotorID = 31;
    private final int kBottomID = 32;

    private final int kShooterCurrentLimit = 30;

    private final TalonFX talonFX1;
    private final TalonFX talonFX2;

    private final TalonFXConfiguration config1;


     public Shooter(CANBus canbus) {
        this.talonFX1 = new TalonFX(kTopMotorID, canbus);
        this.talonFX2 = new TalonFX(kBottomID, canbus);
        this.config1 = new TalonFXConfiguration();
        config1.CurrentLimits.SupplyCurrentLimit = kShooterCurrentLimit;
        config1.CurrentLimits.SupplyCurrentLimitEnable = true;


        config1.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config1.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        talonFX1.getConfigurator().apply(config1);
        talonFX2.getConfigurator().apply(config1);
    }

    public void setVoltageLeft(double voltage) {
        talonFX1.setVoltage(voltage);
        talonFX2.setVoltage(-voltage);
    }

    public void setVoltageRight(double voltage) {
        talonFX2.setVoltage(voltage);
    }

    public Command runVoltageLeft(double voltage) {
        return this.runEnd(() -> setVoltageLeft(voltage), () -> stop());
    }

    public Command runVoltageRight(double voltage) {
        return this.runEnd(() -> setVoltageRight(voltage), () -> stop());
    }

    public void stop() {
        talonFX1.stopMotor();
        talonFX2.stopMotor();
    }

}
