package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.FaultLogger;

import static frc.robot.Ports.Elevator.*;

public class Elevator extends SubsystemBase {
  // TalonFX configuration
  private final int kElevatorCurrentLimit = 5;

  // Limit switch channels
  // TODO: replace these references with those listed in Ports.java
  private final int kMinLimitSwitchChannel = 1;
  private final int kMaxLimitSwitchChannel = 2;

  // Talon FX motors
  private TalonFX elevatorLeader;
  private TalonFX elevatorFollower;

  // Talon FX Feedforward
  private final int kS = 0;
  private final int kV = 0;
  private final int kA = 0;

  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS, kV, kA);

  // Talon FX PID
  private final int kP = 0;
  private final int kI = 0;
  private final int kD = 0;
  private final int kTolerance = 10;

  private final PIDController pid = new PIDController(kP, kI, kD);

  // Elevator limit switches
  private DigitalInput minLimitSwitch;
  private DigitalInput maxLimitSwitch;

  // Booleans
  private boolean reachedMaxHeight = false;
  private boolean reachedMinHeight = false;

  public Elevator() {
    // Setting up motors (actuators) for moving the elevator
    elevatorLeader = new TalonFX(ELEVATOR_FOLLOWER);
    elevatorFollower = new TalonFX(ELEVATOR_LEADER);

    // Set secondary motor as follower; should rotate opposed to primary motor
    elevatorFollower.setControl(new Follower(ELEVATOR_LEADER, MotorAlignmentValue.Opposed));

    // TODO: Implement PID
    pid.setTolerance(kTolerance);

    FaultLogger.register(elevatorLeader);
    FaultLogger.register(elevatorFollower);

  }
  public void setElevatorVoltage(double voltage) {
    elevatorLeader.setVoltage(voltage);
  }

  public void stopMotors() {
    elevatorLeader.stopMotor();
    elevatorFollower.stopMotor();
  }

  // TODO: implement code that extends elevator until it hits a limit switch

  public Command moveElevator(double voltage) {
    return this.runEnd(() -> setElevatorVoltage(voltage), () -> stopMotors());
  }
}
