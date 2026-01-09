// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.constants;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class DriveConstants {
    public static final boolean GYRO_INVERT = false; //CCW+, CW-
    public static final int PIGEON_ID = 25;

    public static final double TRACK_WIDTH = Units.inchesToMeters(0); //robot specific
    public static final double WHEEL_BASE = Units.inchesToMeters(0); //robot specific
    public static final double MIN_THROTTLE_LEVEL = 0.2; 
    public static final double DEADBAND = 0.1; 
    public static final double MAX_VELOCITY_METERS_PER_SEC = 24.0;
    public static final double MAX_ANGULAR_VELOCITY_RAD_PER_SEC = MAX_VELOCITY_METERS_PER_SEC / Math.hypot(TRACK_WIDTH * 0.5, WHEEL_BASE * 0.5); 
    public static final double ODOMETRY_FREQ = 100.0; //Hz

    public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH * 0.5, WHEEL_BASE * 0.5);
    
    public static final SensorDirectionValue CANCODER_INVERT = SensorDirectionValue.CounterClockwise_Positive; 
    public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2.0); //tune to your swerve modules
    public static final double ANGLE_GEAR_RATIO = (12.8 / 1.0);
    public static final double DRIVE_GEAR_RATIO = (5.14 / 1.0); 

    //velocity and position scaling factors
    public static final double ANGLE_ENC_ROT2RAD = 2.0 * Math.PI; 

    
    public static final SwerveDriveKinematics SWERVE_KINEMATICS = 
        new SwerveDriveKinematics(
            new Translation2d(TRACK_WIDTH * 0.5, WHEEL_BASE * 0.5), //FL
            new Translation2d(TRACK_WIDTH * 0.5, -WHEEL_BASE * 0.5), //FR
            new Translation2d(-TRACK_WIDTH * 0.5, WHEEL_BASE * 0.5), //BL
            new Translation2d(-TRACK_WIDTH * 0.5, -WHEEL_BASE * 0.5) //BR
        );

    /*FL Mod 0 */
    public static final class Mod0 {
        public static final int DRIVE_MOTOR_ID = 2; 
        public static final int ANGLE_MOTOR_ID = 11; 
        public static final int CANCODER_ID = 21; 
    }

    /*FR Mod 1 */
    public static final class Mod1 {
        public static final int DRIVE_MOTOR_ID = 3; 
        public static final int ANGLE_MOTOR_ID = 4; 
        public static final int CANCODER_ID = 22; 
    }

    /*BL Mod 2 */
    public static final class Mod2 {
        public static final int DRIVE_MOTOR_ID = 5; 
        public static final int ANGLE_MOTOR_ID = 6; 
        public static final int CANCODER_ID = 23; 
    }

    /*BR Mod 3 */
    public static final class Mod3 {
        public static final int DRIVE_MOTOR_ID = 7; 
        public static final int ANGLE_MOTOR_ID = 8; 
        public static final int CANCODER_ID = 24; 
    }


    //Drive config 
    public static final double DRIVE_Kp = 0.0; 
    public static final double DRIVE_Ki = 0.0;
    public static final double DRIVE_Kd = 0.0; 
    public static final double DRIVE_Ks = 0.043; 
    public static final double DRIVE_Kv = 0.0198; 
    public static final InvertedValue DRIVE_MOTOR_INVERT = InvertedValue.CounterClockwise_Positive; 
    public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Coast;
    public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;  
    public static final int DRIVE_CURRENT_LIMIT = 60; 
    public static final int DRIVE_CURRENT_LOWER = 40;
    public static final double DRIVE_CURRENT_LOWER_TIME = 1.0; 
    public static final double RAMP = 0.0; 
    
    //Angle config
    public static final double ANGLE_Kp = 2.0; 
    public static final double ANGLE_Ki = 0.0; 
    public static final double ANGLE_Kd = 0.0; 
    public static final InvertedValue ANGLE_MOTOR_INVERT = InvertedValue.CounterClockwise_Positive; 
    public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Coast; 
    public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true; 
    public static final int ANGLE_CURRENT_LIMIT = 50; 
    public static final int ANGLE_CURRENT_LOWER = 40;
    public static final double ANGLE_CURRENT_LOWER_TIME = 1.0; 

}
