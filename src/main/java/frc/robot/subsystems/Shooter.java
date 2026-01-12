package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;
// import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

    private final int kTopID = 31;
    private final int kBottomID = 32;

    private final int kShooterCurrentLimit = 30;

    private final TalonFX upperMotor;
    private final TalonFX lowerMotor;

    private final TalonFXConfiguration config1;


     public Shooter() {
        this.upperMotor = new TalonFX(kTopID);
        this.lowerMotor = new TalonFX(kBottomID);
        this.config1 = new TalonFXConfiguration();
        config1.CurrentLimits.SupplyCurrentLimit = kShooterCurrentLimit;
        config1.CurrentLimits.SupplyCurrentLimitEnable = true;


        config1.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config1.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        upperMotor.getConfigurator().apply(config1);
        lowerMotor.getConfigurator().apply(config1);


        // should in theory force the lower motor to follow the top motor?
        lowerMotor.setControl(new Follower(kBottomID, MotorAlignmentValue.Opposed));
        
    }

    public void stopMotors() {
        upperMotor.stopMotor();
        lowerMotor.stopMotor();
    }
    
    public void setShooterVoltage(double voltage) {
        upperMotor.setVoltage(voltage);
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
