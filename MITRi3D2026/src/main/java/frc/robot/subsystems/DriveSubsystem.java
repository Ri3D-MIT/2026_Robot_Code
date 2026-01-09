// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.SwerveModule;
import frc.lib.util.constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  public SwerveDriveOdometry swerveOdometry;
  public frc.lib.util.SwerveModule[] m_SwerveMods;

  public DriveSubsystem() {
            m_SwerveMods = new SwerveModule[] {
                new SwerveModule(0,
                    DriveConstants.Mod0.ANGLE_MOTOR_ID,
                    DriveConstants.Mod0.DRIVE_MOTOR_ID,
                    DriveConstants.Mod0.CANCODER_ID, 
                    0.4),
                new SwerveModule(1, 
                    DriveConstants.Mod1.ANGLE_MOTOR_ID,
                    DriveConstants.Mod1.DRIVE_MOTOR_ID,
                    DriveConstants.Mod1.CANCODER_ID, 
                    0.4),
                new SwerveModule(2,
                    DriveConstants.Mod2.ANGLE_MOTOR_ID,
                    DriveConstants.Mod2.DRIVE_MOTOR_ID,
                    DriveConstants.Mod2.CANCODER_ID, 
                    0.4),
                new SwerveModule(3,
                    DriveConstants.Mod3.ANGLE_MOTOR_ID,
                    DriveConstants.Mod3.DRIVE_MOTOR_ID,
                    DriveConstants.Mod3.CANCODER_ID,
                    0.4)
        };
        for (SwerveModule mod : m_SwerveMods) {
            mod.reset();
        }

  }
    /**
     * 
     * @param translation     X and Y values (multiply by max speed)
     * @param rotation        Theta value (multiply by max angular velocity)
     * @param currentRotation The current rotation/theta of the robot (i.e. the yaw
     *                        from the gyro)
     * @param fieldRelative   Is field relative
     * @param isOpenLoop
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.MAX_VELOCITY_METERS_PER_SEC);
    
        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
          m_SwerveMods[i].setDesiredState(setpointStates[i]);
        }
      }

    public void drive(Translation2d translation, double rotation, Rotation2d currentRotation, boolean fieldRelative,
            boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        translation.getX(),
                        translation.getY(),
                        rotation,
                        currentRotation)
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));

        setModuleStates(swerveModuleStates);
    }


    public void drive(ChassisSpeeds chassisSpeeds) { //for auton
        SwerveModuleState[] swerveModuleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(
                chassisSpeeds);
        setModuleStates(swerveModuleStates);
    }

    /**
     * Overloaded drive method.
     * @param chassisSpeeds to drive at.
     * @param speedCapTranslation in m/s.
     * @param speedCapRotation in rad/s.
     */
    public void drive(ChassisSpeeds chassisSpeeds, double speedCapTranslation, double speedCapRotation) {
        SwerveModuleState[] swerveModuleStates = DriveConstants.SWERVE_KINEMATICS.toSwerveModuleStates(
                chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, chassisSpeeds, speedCapTranslation,
                speedCapTranslation, speedCapRotation);
        setModuleStates(swerveModuleStates);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_VELOCITY_METERS_PER_SEC);

        for (SwerveModule mod : m_SwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber]);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : m_SwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public ChassisSpeeds getChassisSpeed() {
        SwerveModuleState[] arr = getModuleStates();
        return DriveConstants.SWERVE_KINEMATICS.toChassisSpeeds(arr); 
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : m_SwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

  @Override
  public void periodic() {
    ChassisSpeeds chassisSpeed = this.getChassisSpeed();
  }
}
