package frc.robot.subsystems;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Ports.Shooter.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
// import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

  // private final FlywheelSim flywheelSim;

  private final double kS = 0.02;
  private final double kV = 0.02;
  private final double kA = 0;

  private final double kP = 0.1;
  private final double kD = 0;

  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS, kV, kA);
  private final PIDController fb = new PIDController(kP, 0, kD);

  // TODO actually set constants

  /** pid tolerance, radians per second */
  private final double TOLERANCE = 10;

  /** max flywheel speed, rads per sec */
  private final double MAX_SPEED = 500;

  private final double SHOOT_SPEED = 400;

  private final MomentOfInertia MOI = KilogramSquareMeters.of(0.01);

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
    bottomMotor.getConfigurator().apply(config);

    // Create Flywheel simulation for shooter.

    // this.flywheelSim =
    //     new FlywheelSim(
    //         LinearSystemId.createFlywheelSystem(
    //             DCMotor.getKrakenX60(2), MOI.in(KilogramSquareMeters), 1),
    //         DCMotor.getKrakenX60(2));

    FaultLogger.register(topMotor);
    FaultLogger.register(bottomMotor);

    fb.setTolerance(TOLERANCE);

    // setDefaultCommand(runShooter(200));
  }

  public void stopMotors() {
    topMotor.stopMotor();
    bottomMotor.stopMotor();
  }

  public void setShooterVoltage(double voltage) {
    topMotor.setVoltage(voltage);
    // flywheelSim.setInputVoltage(voltage);
  }

  /** current flywheel vel in radians per second */
  // @Logged
  // public double velocity() {
  //   return Robot.isReal()
  //       ? topMotor.getVelocity().getValue().in(RadiansPerSecond)
  //       : flywheelSim.getAngularVelocityRadPerSec();
  // }

  /** updates velocity goal, radians per second */
  // public void updateGoal(double velocity) {
  //   double goal = Double.isNaN(velocity) ? 0 : MathUtil.clamp(velocity, -MAX_SPEED, MAX_SPEED);
  //   // TODO make sure units check out
  //   double current_velocity = velocity();
  //   double voltage =
  //       fb.calculate(current_velocity, velocity)
  //           + ff.calculateWithVelocities(current_velocity, goal);
  //   setShooterVoltage(voltage);
  // }

  // public Command runShooter(DoubleSupplier velocity) {
  //   return run(() -> updateGoal(velocity.getAsDouble()));
  // }

  // public Command runShooter(double velocity) {
  //   return runShooter(() -> velocity);
  // }

  // public Command shoot() {
  //   return runShooter(SHOOT_SPEED);
  // }

  public Command shoot() {
    return this.runEnd(() -> setShooterVoltage(SHOOT_SPEED), () -> stopMotors());
  }

  // public Test goToTest(double velocity) {
  //   Command testCommand =
  //       runShooter(velocity)
  //           .until(fb::atSetpoint)
  //           .withTimeout(10)
  //           .withName("Shooter Test: go to " + velocity + " radians per sec");
  //   EqualityAssertion atGoal =
  //       Assertion.eAssert("flywheel speed", () -> velocity, this::velocity, TOLERANCE);
  //   return new Test(testCommand, Set.of(atGoal));
  // }

  // @Override
  // public void periodic() {
  //   flywheelSim.update(0.02);
  // }

  @Override
  public void close() throws Exception {
    topMotor.close();
    bottomMotor.close();
  }
}
