package frc.robot.subsystems;

import static frc.robot.Ports.Shooter.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
// import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.FaultLogger;

public class Shooter extends SubsystemBase {
  private final int kShooterCurrentLimit = 30;

  private final TalonFX topMotor;
  private final TalonFX bottomMotor;

  private final double kS = 0;
  private final double kV = 0;
  private final double kA = 0;

  private final double kP = 0;
  private final double kD = 0;

  private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(kS, kV, kA);
  private final PIDController fb = new PIDController(kP, 0, kD);

  // TODO actually set constants

  /** pid tolerance, radians per second */
  private final double TOLERANCE = 10;

  /** max flywheel speed, rads per sec */
  private final double MAX_SPEED = 100;

  public Shooter() {
    this.topMotor = new TalonFX(TOP_LEADER);
    this.bottomMotor = new TalonFX(BOTTOM_FOLLOWER);

    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = kShooterCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    // TODO gear ratio?
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    topMotor.getConfigurator().apply(config);
    bottomMotor.getConfigurator().apply(config);

    bottomMotor.setControl(new Follower(TOP_LEADER, MotorAlignmentValue.Opposed));

    FaultLogger.register(topMotor);
    FaultLogger.register(bottomMotor);

    fb.setTolerance(TOLERANCE);
  }

  public void stopMotors() {
    topMotor.stopMotor();
    bottomMotor.stopMotor();
  }

  public void setShooterVoltage(double voltage) {
    topMotor.setVoltage(voltage);
  }

  /** updates velocity goal, radians per second */
  public void updateGoal(double velocity) {
    double goal = Double.isNaN(velocity) ? 0 : MathUtil.clamp(velocity, -MAX_SPEED, MAX_SPEED);
    // TODO make sure units check out
    double current_velocity = topMotor.getVelocity().getValueAsDouble();

    setShooterVoltage(
        fb.calculate(current_velocity) + ff.calculateWithVelocities(current_velocity, goal));
  }

  public Command runShooter(DoubleSupplier velocity) {
    return run(() -> updateGoal(velocity.getAsDouble())).finallyDo(() -> setShooterVoltage(0));
  }

  public Command runShooter(double velocity) {
    return runShooter(() -> velocity);
  }
}
