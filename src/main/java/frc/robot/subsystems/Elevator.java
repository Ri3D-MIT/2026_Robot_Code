package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  // TalonFX IDs
  private final int kElevatorPrimaryID = 37;
  private final int kElevatorSecondaryID = 38;

  // TalonFX configuration
  private final int kElevatorCurrentLimit = 5;

  // Limit switch channels
  private final int kMinLimitSwitchChannel = 1;
  private final int kMaxLimitSwitchChannel = 2;

  // Talon FX motors
  private TalonFX elevatorPrimary;
  private TalonFX elevatorSecondary;

  // Elevator limit switches
  private DigitalInput minLimitSwitch;
  private DigitalInput maxLimitSwitch;

  // Booleans
  private boolean reachedMaxHeight = false;
  private boolean reachedMinHeight = false;

  public Elevator() {
    // Setting up motors (actuators) for moving the elevator
    elevatorPrimary = new TalonFX(kElevatorPrimaryID);
    elevatorSecondary = new TalonFX(kElevatorSecondaryID);

    // Set secondary motor as follower; should rotate opposed to primary motor
    elevatorSecondary.setControl(new Follower(kElevatorPrimaryID, MotorAlignmentValue.Opposed));
  }

  public void setElevatorVoltage(double voltage) {
    elevatorPrimary.setVoltage(voltage);
  }

  public void stopMotors() {
    elevatorPrimary.stopMotor();
    elevatorSecondary.stopMotor();
  }

  public Command moveElevator(double voltage) {
    return this.runEnd(() -> setElevatorVoltage(voltage), () -> stopMotors());
  }
}
