// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.PoseEstimationSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DefaultDriveCommand extends Command {
  /** Creates a new DefaultDriveCommand. */
  DriveSubsystem driveSubsystem;
  PoseEstimationSubsystem poseEstimationSubsystem;
  DoubleSupplier xSupplier;
  DoubleSupplier ySupplier;
  DoubleSupplier thetaSupplier;
  DoubleSupplier throttle;
  boolean fieldRelative;

  public DefaultDriveCommand(
    DriveSubsystem ds, 
    PoseEstimationSubsystem pes, 
    DoubleSupplier x,
    DoubleSupplier y, 
    DoubleSupplier theta,
    DoubleSupplier throt,
    boolean fR
  ) {

    driveSubsystem = ds; 
    poseEstimationSubsystem = pes; 
    xSupplier = x;
    ySupplier = y; 
    thetaSupplier = theta; 
    fieldRelative = fR;
    throttle = throt;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
    addRequirements(poseEstimationSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double slope = 1 - DriveConstants.MIN_THROTTLE_LEVEL;
    double scale = (slope * throttle.getAsDouble() + DriveConstants.MIN_THROTTLE_LEVEL);
    Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

     // Apply rotation deadband
    double theta = MathUtil.applyDeadband(thetaSupplier.getAsDouble(), DriveConstants.DEADBAND); 

    // Square rotation value for more precise control
    theta = Math.copySign(theta * theta, theta);

    boolean isFlipped =
      DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == Alliance.Red;

    // Convert to field relative speeds & send command

    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * scale * DriveConstants.MAX_VELOCITY_METERS_PER_SEC /** m_directionInvert*/,
            linearVelocity.getY() * scale * DriveConstants.MAX_VELOCITY_METERS_PER_SEC /** m_directionInvert*/,
            theta * scale * DriveConstants.MAX_ANGULAR_VELOCITY_RAD_PER_SEC);

          if (fieldRelative)
          {
            driveSubsystem.runVelocity(
            ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped
                    ? poseEstimationSubsystem.getPose().getRotation().plus(new Rotation2d(Math.PI))
                    : poseEstimationSubsystem.getPose().getRotation())

                );
            } 
            else {
              driveSubsystem.runVelocity(speeds);
          } 

    //logging
    SmartDashboard.putNumber("x axis", xSupplier.getAsDouble()); 
    SmartDashboard.putNumber("y axis", ySupplier.getAsDouble()); 
    SmartDashboard.putNumber("rot axis", thetaSupplier.getAsDouble()); 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.drive(new Translation2d(), 0, new Rotation2d(), fieldRelative, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DriveConstants.DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }
}
