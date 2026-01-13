package frc.robot.subsystems;

import static frc.robot.Ports.Feeder.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.FaultLogger;

public class Feeder extends SubsystemBase{
    private final TalonFX feederMotor;
    private final TalonFXConfiguration feederConfiguration;

    private int kFeederCurrentLimit = 15;

    public Feeder() {
        feederMotor = new TalonFX(FEEDER);
        feederConfiguration = new TalonFXConfiguration();
        
        feederConfiguration.CurrentLimits.SupplyCurrentLimit = kFeederCurrentLimit;
        feederConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

        feederConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        feederConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        feederMotor.getConfigurator().apply(feederConfiguration);

        FaultLogger.register(feederMotor);
    }

    public void stopFeeder() {
        feederMotor.stopMotor();
    }

    public void setVoltageFeeder(double voltage) {
        feederMotor.setVoltage(voltage);
    }

    public Command runFeeder(double voltage) {
        return this.runEnd(() -> setVoltageFeeder(voltage), () -> stopFeeder());
    }

}
