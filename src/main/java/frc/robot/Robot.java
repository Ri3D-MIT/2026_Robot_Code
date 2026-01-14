// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.*;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
// import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.CommandRobot;
import frc.lib.FaultLogger;
import frc.robot.subsystems.Intake;
import frc.lib.Test;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

@Logged
public class Robot extends CommandRobot {
  /* log and replay timestamp and joystick data */
  private final HootAutoReplay m_timeAndJoystickReplay =
      new HootAutoReplay().withTimestampReplay().withJoystickReplay();

  private double MaxSpeed =
      1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.25)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(
              DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  public final SwerveDrive drivetrain = TunerConstants.createDrivetrain();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  @Logged private final Shooter shooter = new Shooter();

  @Logged public final Feeder feeder = new Feeder();

  private final CommandXboxController joystick = new CommandXboxController(0);

  // @Logged private final Elevator elevator = new Elevator();

  public Robot() {
    super(0.02);
    configureGameBehavior();
    configureBindings();
  }

  public void configureGameBehavior() {
    addPeriodic(FaultLogger::update, 2);
    // Epilogue.bind(this);
  }

  public void configureBindings() {
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        -joystick.getLeftY() * 0.5) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * 0.5) // Drive left with negative X (left)
                    .withRotationalRate(
                        -joystick.getRightX()
                            * 0.5) // Drive counterclockwise with negative X (left)
            ));

    final var idle = new SwerveRequest.Idle();
    disabled().whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));
    joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    drivetrain.registerTelemetry(logger::telemeterize);
    joystick.leftTrigger().whileTrue(shooter.runShooter(()->10));
    joystick.rightTrigger().whileTrue(feeder.feed());

    // joystick.y().whileTrue(elevator.goTo(0.2));

    // test()
    //     .whileTrue(
    //         Test.toCommand(elevator.goToTest(Inches.of(10)), elevator.goToTest(Inches.of(0))));
  }
  public void configureBindings() {
    joystick.x().onTrue(intake.ground());
    joystick.b().onTrue(intake.stow());

    joystick.rightBumper().toggleOnTrue(intake.run(() -> intake.setIntakeVoltage(4)));
  }

  @Override
  public void robotPeriodic() {
    m_timeAndJoystickReplay.update();
    CommandScheduler.getInstance().run();
  }
}
