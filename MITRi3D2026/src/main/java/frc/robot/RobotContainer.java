// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;

public class RobotContainer {
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final PoseEstimationSubsystem poseEstimationSubsystem = new PoseEstimationSubsystem(driveSubsystem);
  private final CommandXboxController driverController =
      new CommandXboxController(0);

  public RobotContainer() {
    configureDriverBindings();
  }

  public void setDefaultCommands(){
    //drive w joysticks + boost right trigger 
    driveSubsystem.setDefaultCommand(
      new DefaultDriveCommand(
          driveSubsystem,
          poseEstimationSubsystem,
          () -> -driverController.getLeftY(),
          () -> -driverController.getLeftX(),
          () -> -driverController.getRightX(),
          () -> driverController.getRightTriggerAxis(), 
          true)
          );
  }

  private void configureDriverBindings() {
     driverController.back().onTrue(poseEstimationSubsystem.zeroAngleCommand());

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
