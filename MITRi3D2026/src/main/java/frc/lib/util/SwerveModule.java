// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.util.constants.DriveConstants;

/** Contains swerve module object to be instantiated in Drivetrain subsystem*/
public class SwerveModule {
    public int moduleNumber; 
    private TalonFX angleMotor; 
    private TalonFX driveMotor;  
    private CANcoder angleEncoder; 
    private ProfiledPIDController anglePidController; 
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);
    private TalonFXConfiguration angleConfig = new TalonFXConfiguration(); 
    private TalonFXConfiguration driveConfig = new TalonFXConfiguration(); 
    private CANcoderConfiguration canCoderConfig = new CANcoderConfiguration(); 


    public SwerveModule(
        int moduleNumber,
        int angleMotorID,
        int driveMotorID,
        int canCoderID,
        double kP){
            this.moduleNumber = moduleNumber;
            angleMotor = new TalonFX(angleMotorID);
            driveMotor = new TalonFX(driveMotorID);
            angleEncoder = new CANcoder(canCoderID);

            config();

            anglePidController = new ProfiledPIDController(kP, 0.0, 0.0, 
                new TrapezoidProfile.Constraints(20*2*Math.PI, 20*2*Math.PI));
            
            anglePidController.enableContinuousInput(-Math.PI, Math.PI);
            driveMotor.setPosition(0.0);
        }

    public void setDesiredState(SwerveModuleState desiredState) { 
        Rotation2d currentAngle = Rotation2d.fromRadians(getTurningPosition());
        desiredState.optimize(currentAngle);  
        desiredState.cosineScale(currentAngle); 
        setAngle(desiredState);
        setDriveVelocity(desiredState.speedMetersPerSecond / DriveConstants.WHEEL_RADIUS_METERS);
    }


    private void setDriveVelocity(double velocityRadPerSec) { //incorporates FF
        double ffVolts = DriveConstants.DRIVE_Ks * Math.signum(velocityRadPerSec) + DriveConstants.DRIVE_Kv * velocityRadPerSec;
        driveVelocity.Velocity = velocityRadPerSec;
        driveVelocity.FeedForward = ffVolts; 
        driveMotor.setControl(driveVelocity);
    }

    private void setAngle(SwerveModuleState desiredState) {
        double desiredPower = anglePidController.calculate(getTurningPosition(),
                desiredState.angle.getRadians()); 
        angleMotor.set(desiredPower);
    }
    

    public void rawSet(double drive, double turn) {
        driveMotor.set(drive);
        angleMotor.set(turn);
    }

    public double getTurningPosition() {
        return angleEncoder.getAbsolutePosition().getValueAsDouble() * DriveConstants.ANGLE_ENC_ROT2RAD; 
    }

    public double getTurningVelocity() {
        return angleEncoder.getVelocity().getValueAsDouble();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getVelocity().getValueAsDouble() * DriveConstants.WHEEL_RADIUS_METERS, new Rotation2d(getTurningPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveMotor.getPosition().getValueAsDouble(), Rotation2d.fromRadians(getTurningPosition()));
    }

    public void reset() {
        driveMotor.setPosition(0);
        anglePidController.reset(0);
    }

    public void stop() {
        angleMotor.stopMotor();
        driveMotor.stopMotor();
    }

    public void config(){
        /* Inverts */
        canCoderConfig.MagnetSensor.SensorDirection = DriveConstants.CANCODER_INVERT;
        angleConfig.MotorOutput.Inverted = DriveConstants.ANGLE_MOTOR_INVERT; 
        driveConfig.MotorOutput.Inverted = DriveConstants.DRIVE_MOTOR_INVERT;

        /* Neutral Mode */
        angleConfig.MotorOutput.NeutralMode = DriveConstants.ANGLE_NEUTRAL_MODE; 
        driveConfig.MotorOutput.NeutralMode = DriveConstants.DRIVE_NEUTRAL_MODE;

        /* Gear ratios and wrapping */
        angleConfig.Feedback.SensorToMechanismRatio = DriveConstants.ANGLE_GEAR_RATIO; 
        driveConfig.Feedback.SensorToMechanismRatio = DriveConstants.DRIVE_GEAR_RATIO;
        angleConfig.ClosedLoopGeneral.ContinuousWrap = true; 

        /* Current Limiting */
        angleConfig.CurrentLimits.SupplyCurrentLimitEnable = DriveConstants.ANGLE_ENABLE_CURRENT_LIMIT; 
        angleConfig.CurrentLimits.SupplyCurrentLowerLimit = DriveConstants.ANGLE_CURRENT_LOWER;
        angleConfig.CurrentLimits.SupplyCurrentLowerTime = DriveConstants.ANGLE_CURRENT_LOWER_TIME;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = DriveConstants.DRIVE_ENABLE_CURRENT_LIMIT; 
        driveConfig.CurrentLimits.SupplyCurrentLimit = DriveConstants.DRIVE_CURRENT_LIMIT; 
        driveConfig.CurrentLimits.SupplyCurrentLowerLimit = DriveConstants.DRIVE_CURRENT_LOWER;
        driveConfig.CurrentLimits.SupplyCurrentLowerTime = DriveConstants.DRIVE_CURRENT_LOWER_TIME;

        /* Angle PID */
        angleConfig.Slot0.kP = DriveConstants.ANGLE_Kp; 
        angleConfig.Slot0.kI = DriveConstants.ANGLE_Ki;
        angleConfig.Slot0.kD = DriveConstants.ANGLE_Kd;

        /* Drive PID */
        driveConfig.Slot0.kP = DriveConstants.DRIVE_Kp; 
        driveConfig.Slot0.kI = DriveConstants.DRIVE_Ki;
        driveConfig.Slot0.kD = DriveConstants.DRIVE_Kd;

        /* Ramping */
        driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = DriveConstants.RAMP; 

        angleEncoder.getConfigurator().apply(canCoderConfig); //apply configurations
        driveMotor.getConfigurator().apply(driveConfig);
        angleMotor.getConfigurator().apply(angleConfig);


    }
    
     


}
