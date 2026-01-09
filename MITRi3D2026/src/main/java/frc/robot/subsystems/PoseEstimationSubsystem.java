// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.constants.DriveConstants;

public class PoseEstimationSubsystem extends SubsystemBase {
  DriveSubsystem driveSubsystem;
  static SwerveDrivePoseEstimator swerveDrivePoseEst;
  public frc.lib.util.SwerveModule[] swerveMods;
  public Pigeon2 pigeon;
  public double angleOffset;
  public PoseEstimationSubsystem(DriveSubsystem driveSubsystem) {
      this.driveSubsystem = driveSubsystem; 
      pigeon = new Pigeon2(DriveConstants.PIGEON_ID);
      angleOffset = 0.0;

      this.zeroGyro();

    swerveDrivePoseEst = new SwerveDrivePoseEstimator(
        DriveConstants.SWERVE_KINEMATICS,
        getGyroYaw(),
        driveSubsystem.getModulePositions(),
        new Pose2d()
        ,VecBuilder.fill(0.1, 0.1, 0.1), //odometry std devs
        VecBuilder.fill(0.9, 0.9, 0.9)
        );
  }

  public void zeroGyro() { //resets gyro 
    pigeon.setYaw(0.0); 
    }

  public void zeroAngle(){ //resets robot angle
    pigeon.setYaw(0.0); 
    swerveDrivePoseEst.resetPosition(getGyroYaw(), driveSubsystem.getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d(angleOffset)));
    }

    public Command zeroAngleCommand(){
    return new InstantCommand(()->this.zeroAngle());
  }

  private Rotation2d getGyroYaw() {
    return (DriveConstants.GYRO_INVERT) ? Rotation2d.fromDegrees(360 - pigeon.getYaw().getValueAsDouble())
        : Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble());
    }

    public void resetOdometry(Pose2d pose) {
    swerveDrivePoseEst.resetPosition(getGyroYaw(), driveSubsystem.getModulePositions(), pose);
  }

  public Pose2d getPose() {
    return swerveDrivePoseEst.getEstimatedPosition();
  }

  public double getPitch() {
    return pigeon.getPitch().getValueAsDouble(); 
  }

  public double getRoll() {
    return pigeon.getRoll().getValueAsDouble();
  }

  public double getYaw() {
    return pigeon.getYaw().getValueAsDouble(); //deg
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    swerveDrivePoseEst.update(getGyroYaw(), driveSubsystem.getModulePositions());
    DriverStation.refreshData();
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    if (alliance.get().equals(DriverStation.Alliance.Red)){ //Alliance is Red
      angleOffset = Math.PI;
    } else{
      angleOffset = 0.0; // Alliance is Blue or not present 
    }
  }
}
