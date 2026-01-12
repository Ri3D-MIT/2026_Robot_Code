package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    // Talon FX motors
    private TalonFX elevatorPrimary;
    private TalonFX elevatorSecondary;


    private int kElevatorPrimaryID = 37;
    private int kElevatorSecondaryID = 38;
    
    public Elevator() {
        // Setting up motors (actuators) for moving the elevator
        elevatorPrimary = new TalonFX(kElevatorPrimaryID);
        elevatorSecondary = new TalonFX(kElevatorSecondaryID);

        // Set secondary motor as follower; should rotate opposed to primary motor
        elevatorSecondary.setControl(new Follower(kElevatorPrimaryID, MotorAlignmentValue.Opposed));
    }


}
