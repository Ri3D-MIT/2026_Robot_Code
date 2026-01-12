package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.hardware.CANcoder;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    // Talon FX motors
    private TalonFX elevator1;
    private TalonFX elevator2;
    private TalonFX elevator3;
    private TalonFX elevator4;

    private int kElevator1ID = 37;
    private int kElevator2ID = 38;
    private int kElevator3ID = 39;
    private int kElevator4ID = 40;
    
    
    public Elevator() {
        elevator1 = new TalonFX(kElevator1ID);
        elevator2 = new TalonFX(kElevator2ID);
        elevator3 = new TalonFX(kElevator3ID);
        elevator4 = new TalonFX(kElevator4ID);



    }
}
