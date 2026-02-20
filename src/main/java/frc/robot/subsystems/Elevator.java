package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static frc.robot.Ports.Elevator.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Assertion;
import frc.lib.Assertion.EqualityAssertion;
import frc.lib.FaultLogger;
import frc.lib.Test;
import frc.robot.Robot;
import java.util.Set;
import java.util.function.DoubleSupplier;

public class Elevator extends SubsystemBase implements AutoCloseable {
  // TalonFX configuration
  private final int kElevatorCurrentLimit = 5;

  // Talon FX motors
  private TalonFX leader;
  private TalonFX follower;

  // Talon FX Feedforward
  private final double kS = 0;
  private final double kV = 0.3;
  private final double kA = 0;
  private final double kG = 0;
  private final double kG_LIFT = -5;

  // kG = [m•g•r]/V

  @Logged private final ElevatorFeedforward ff = new ElevatorFeedforward(kS, kG, kV, kA);
  private final ElevatorFeedforward ffLift = new ElevatorFeedforward(kS, kG_LIFT, kV, kA);

  // TODO gear ratio and mech ratio

  private final double kP = 2;
  private final double kI = 0;
  private final double kD = 0;
  private final double kTolerance = Inches.of(0.3).in(Meters);

  private final LinearVelocity MAX_SPEED = MetersPerSecond.of(1);
  private final LinearAcceleration MAX_ACCELERATION = MetersPerSecondPerSecond.of(1);

  private final ProfiledPIDController pid =
      new ProfiledPIDController(
          kP,
          kI,
          kD,
          new Constraints(
              MAX_SPEED.in(MetersPerSecond), MAX_ACCELERATION.in(MetersPerSecondPerSecond)));

  // TODO set constants
  private final Distance MAX_EXTENSION = Meters.of(0.444);
  private final Distance DRUM_RADIUS = Inches.of(0.25).times(22 / (2 * Math.PI));
  private final Mass MASS = Pounds.of(5);

  private final ElevatorSim elevatorSim;

  public Elevator() {
    // Setting up motors (actuators) for moving the elevator
    leader = new TalonFX(ELEVATOR_FOLLOWER);
    follower = new TalonFX(ELEVATOR_LEADER);

    var config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = kElevatorCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.RotorToSensorRatio = 1;

    config.Feedback.SensorToMechanismRatio = 20;
    leader.setPosition(0);

    leader.getConfigurator().apply(config);
    follower.getConfigurator().apply(config);

    // Set secondary motor as follower; should rotate opposed to primary motor
    follower.setControl(new Follower(ELEVATOR_LEADER, MotorAlignmentValue.Opposed));

    pid.setTolerance(kTolerance);

    FaultLogger.register(leader);
    FaultLogger.register(follower);

    elevatorSim =
        new ElevatorSim(
            LinearSystemId.createElevatorSystem(
                DCMotor.getKrakenX60(2), MASS.in(Kilograms), DRUM_RADIUS.in(Meters), 20),
            DCMotor.getKrakenX60(2),
            0,
            MAX_EXTENSION.in(Meters),
            true,
            0);

    setDefaultCommand(stop().andThen(idle()));

    new Trigger(() -> (position() <= kTolerance) || (position() >= MAX_EXTENSION.in(Meters)))
        .onTrue(stop());
  }

  private double drumRotationsToMeters(double rotations) {
    return rotations * 2.0 * Math.PI * DRUM_RADIUS.in(Meters);
  }

  private double drumRpsToMetersPerSecond(double rps) {
    return rps * 2.0 * Math.PI * DRUM_RADIUS.in(Meters);
  }

  public void setElevatorVoltage(double voltage) {
    leader.setVoltage(voltage);
    elevatorSim.setInputVoltage(voltage);
  }

  /** position meters */
  @Logged
  public double position() {
    return Robot.isReal()
        ? drumRotationsToMeters(leader.getPosition().getValueAsDouble())
        : elevatorSim.getPositionMeters();
  }

  /** velocity meters per sec (theoretically) */
  public double velocity() {
    return Robot.isReal()
        ? drumRpsToMetersPerSecond(leader.getVelocity().getValueAsDouble())
        : elevatorSim.getVelocityMetersPerSecond();
  }

  public void updateSetpoint(double position, ElevatorFeedforward ffControl) {
    var lastState = pid.getSetpoint();
    double pidVoltage = pid.calculate(position(), Double.isNaN(position) ? position() : position);
    var nextState = pid.getSetpoint();
    // double ffVoltage = ff.calculateWithVelocities(lastState.velocity, nextState.velocity);
    double ffVoltage = ffControl.calculateWithVelocities(lastState.velocity, nextState.velocity);

    setElevatorVoltage(pidVoltage + ffVoltage);
  }

  public void updateSetpoint(double position) {
    updateSetpoint(position, ff);
  }

  public Command goTo(DoubleSupplier extension) {
    return run(() -> updateSetpoint(extension.getAsDouble()))
        .finallyDo(() -> setElevatorVoltage(0));
  }

  public Command goTo(double extension) {
    return goTo(() -> extension);
  }

  public Test goToTest(Distance height) {
    Command testCommand =
        goTo(height.in(Meters)).until(pid::atGoal).withTimeout(7)
        .andThen(Commands.runOnce(() -> System.out.println("exp: " + height.in(Meters) + "; real: " + this.position())))
        .withName("elevator test");
    EqualityAssertion atGoal =
        Assertion.eAssert("elevator height", () -> height.in(Meters), this::position, kTolerance);
    EqualityAssertion stayingAtGoal =
        Assertion.eAssert("elevator vel", () -> 0, this::velocity, 0.1);
    return new Test(testCommand, Set.of(atGoal, stayingAtGoal));
  }

  public Command stop() {
    return Commands.runOnce(() -> setElevatorVoltage(0));
  }

  public Command lift() {
    return run(() -> updateSetpoint(0, ffLift)).finallyDo(() -> setElevatorVoltage(0));
  }

  @Override
  public void close() throws Exception {
    leader.close();
    follower.close();
  }

  @Override
  public void periodic() {
    elevatorSim.update(0.02);
  }
}
