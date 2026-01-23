package frc.robot.subsystems;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static frc.robot.Ports.Shooter.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
// import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
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

public class Shooter extends SubsystemBase implements AutoCloseable {
  private final int kShooterCurrentLimit = 30;

  private final TalonFX topMotor;
  private final TalonFX bottomMotor;

  private final FlywheelSim flywheelSim;

  private final double kS = 0;
  private final double kV = 0.02;
  private final double kA = 0;

  private final double kP = 0.025;
  private final double kD = 0;

  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS, kV, kA);
  private final PIDController fbTop = new PIDController(kP, 0, kD);
  private final PIDController fbBottom = new PIDController(kP * 1.5, 0, kD);

  // TODO actually set constants

  /** pid tolerance, radians per second */
  public static final double TOLERANCE = 10;

  /** max flywheel speed, rads per sec */
  public static final AngularVelocity MAX_SPEED = RadiansPerSecond.of(600);

  public static final AngularVelocity SHOOT_SPEED = RadiansPerSecond.of(400);

  public static final AngularVelocity IDLE_SPEED = RadiansPerSecond.of(400);

  public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.005);

  public Shooter() {
    /*
     * The shooter launches any fuel stored on the robot's hopper.
     *
     * The shooter is powered by two Kraken X60 motors: one on top and one on the bottom.
     * These Kraken X60 motors are powered by a Talon FX motor controller.
     */

    // Creates TalonFX objects corresponding to each motor.
    // All CAN IDs are saved in Ports.java
    this.topMotor = new TalonFX(TOP_LEADER);
    this.bottomMotor = new TalonFX(BOTTOM_FOLLOWER);

    bottomMotor.setControl(new Follower(TOP_LEADER, MotorAlignmentValue.Opposed));

    // Create configuration object for BOTH shooter motors.
    // We bring down the current limit to prevent overloading our electrical circuitry
    // We also set the neutral mode to coast (the shooter motors spool fast :O)
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = kShooterCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    topMotor.getConfigurator().apply(config);

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    bottomMotor.getConfigurator().apply(config);

    // Create Flywheel simulation for shooter.

    this.flywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(2), MOI.in(KilogramSquareMeters), 1),
            DCMotor.getKrakenX60(2));

    FaultLogger.register(topMotor);
    FaultLogger.register(bottomMotor);

    fbTop.setTolerance(TOLERANCE);
    fbBottom.setTolerance(TOLERANCE);

    setDefaultCommand(runShooter(IDLE_SPEED.in(RadiansPerSecond)));
  }

  public Command stopMotors() {
    return Commands.runOnce(
            () -> {
              topMotor.stopMotor();
              bottomMotor.stopMotor();

              flywheelSim.setInputVoltage(0);
            })
        .andThen(idle());
  }

  /** current (top) flywheel vel in radians per second */
  @Logged
  public double velocity() {
    return Robot.isReal()
        ? topMotor.getVelocity().getValue().in(RadiansPerSecond)
        : flywheelSim.getAngularVelocityRadPerSec();
  }

  /** current (top) flywheel acceleration in radians per second square */
  public double acceleration() {
    return Robot.isReal()
        ? topMotor.getAcceleration().getValue().in(RadiansPerSecondPerSecond)
        : flywheelSim.getAngularAccelerationRadPerSecSq();
  }

  private double calculateVoltage(TalonFX motor, PIDController fb, double velocity) {
    double goal =
        Double.isNaN(velocity)
            ? 0
            : MathUtil.clamp(
                velocity, -MAX_SPEED.in(RadiansPerSecond), MAX_SPEED.in(RadiansPerSecond));
    double currentVelocity =
        Robot.isReal()
            ? motor.getVelocity().getValue().in(RadiansPerSecond)
            : flywheelSim.getAngularVelocityRadPerSec();
    return fb.calculate(currentVelocity, goal) + ff.calculateWithVelocities(currentVelocity, goal);
  }

  /** updates velocity goal, radians per second */
  public void update(double topVelocity, double bottomVelocity) {
    double topVoltage = calculateVoltage(topMotor, fbTop, topVelocity);
    double bottomVoltage = calculateVoltage(bottomMotor, fbBottom, bottomVelocity);

    topMotor.setVoltage(topVoltage);
    bottomMotor.setVoltage(bottomVoltage);

    flywheelSim.setInputVoltage(topVoltage);
  }

  public Command runShooter(DoubleSupplier velocity) {
    return run(() -> update(velocity.getAsDouble(), velocity.getAsDouble() * 1.5));
  }

  public Command runShooter(double velocity) {
    return runShooter(() -> velocity);
  }

  public Command shoot() {
    return runShooter(SHOOT_SPEED.in(RadiansPerSecond));
  }

  public Test goToTest(double velocity) {
    Command testCommand =
        runShooter(velocity)
            .until(new Trigger(fbTop::atSetpoint).debounce(1))
            .withTimeout(10)
            .withName("Shooter Test: go to " + velocity + " radians per sec");
    EqualityAssertion atGoal =
        Assertion.eAssert("flywheel speed", () -> velocity, this::velocity, TOLERANCE);
    EqualityAssertion stayingAtGoal =
        Assertion.eAssert("flywheel acceleration", () -> 0, this::acceleration, 50);
    return new Test(testCommand, Set.of(atGoal, stayingAtGoal));
  }

  @Override
  public void periodic() {
    flywheelSim.update(0.02);
  }

  @Override
  public void close() throws Exception {
    topMotor.close();
    bottomMotor.close();
  }
}
