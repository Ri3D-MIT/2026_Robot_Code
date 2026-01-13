// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.*;

import com.ctre.phoenix6.HootAutoReplay;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.CommandRobot;
import frc.lib.FaultLogger;
import frc.lib.Test;
import frc.robot.subsystems.Elevator;

@Logged
public class Robot extends CommandRobot {
  /* log and replay timestamp and joystick data */
  private final HootAutoReplay m_timeAndJoystickReplay =
      new HootAutoReplay().withTimestampReplay().withJoystickReplay();

  private final CommandXboxController joystick = new CommandXboxController(0);

  @Logged private final Elevator elevator = new Elevator();

  public Robot() {
    super(0.02);
    configureGameBehavior();
    configureBindings();
  }

  public void configureGameBehavior() {
    addPeriodic(FaultLogger::update, 2);
    Epilogue.bind(this);
  }

  public void configureBindings() {
    joystick.y().whileTrue(elevator.goTo(0.2));

    test()
        .whileTrue(
            Test.toCommand(elevator.goToTest(Inches.of(10)), elevator.goToTest(Inches.of(0))));
  }

  @Override
  public void robotPeriodic() {
    m_timeAndJoystickReplay.update();
    CommandScheduler.getInstance().run();
  }
}
