// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.SwerveConstants;

public final class SwerveModule {
    private final SparkMax driveMotor;
    private final SparkMax steerMotor;

    private final RelativeEncoder driveMotorEncoder;
    private final RelativeEncoder steerMotorEncoder;

    private final double motorOffsetRadians;
    private final boolean isAbsoluteEncoderReversed;
    private final boolean motor_inv;
    private final CANcoder absoluteEncoder;
    private final PIDController steerPID;

    public SwerveModule(int steerCanID, int driveCanID, int absoluteEncoderPort, double motorOffsetRadians,
            boolean isAbsoluteEncoderReversed, boolean motorReversed) {
        driveMotor = new SparkMax(driveCanID, MotorType.kBrushless);
        steerMotor = new SparkMax(steerCanID, MotorType.kBrushless);
        absoluteEncoder = new CANcoder(absoluteEncoderPort);

        this.motor_inv = motorReversed;
        driveMotorEncoder = driveMotor.getEncoder();
        steerMotorEncoder = steerMotor.getEncoder();

        CANcoderConfiguration cfg = new CANcoderConfiguration();
        cfg.MagnetSensor = new MagnetSensorConfigs();

        cfg.MagnetSensor.MagnetOffset = 0.0;
        absoluteEncoder.getConfigurator().apply(cfg);

        this.motorOffsetRadians = motorOffsetRadians;
        this.isAbsoluteEncoderReversed = isAbsoluteEncoderReversed;

        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig.encoder.positionConversionFactor(SwerveConstants.DRIVE_ROTATION_TO_METER);
        driveConfig.encoder.velocityConversionFactor(SwerveConstants.DRIVE_METERS_PER_MINUTE);
        driveMotor.configure(driveConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig steerConfig = new SparkMaxConfig();
        steerConfig.encoder.positionConversionFactor(SwerveConstants.STEER_ROTATION_TO_RADIANS);
        steerConfig.encoder.velocityConversionFactor(SwerveConstants.STEER_RADIANS_PER_MINUTE);
        steerMotor.configure(steerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        brakeMode();

        steerPID = new PIDController(SwerveConstants.MODULE_KP, 0, SwerveConstants.MODULE_KD);
        steerPID.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveMotorEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveMotorEncoder.getVelocity();
    }

    public double getSteerPosition() {
        return steerMotorEncoder.getPosition();
    }

    public double getSteerVelocity() {
        return steerMotorEncoder.getVelocity();
    }

    public double getAbsoluteEncoderPosition() {
        double angle = Units.rotationsToRadians(absoluteEncoder.getPosition().getValueAsDouble());
        angle -= motorOffsetRadians;
        return angle * (isAbsoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveMotorEncoder.setPosition(0);
        steerMotorEncoder.setPosition(getAbsoluteEncoderPosition());
    }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(-getSteerPosition()));
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getDrivePosition(),
                new Rotation2d(-getSteerPosition()).rotateBy(DriveConstants.NAVX_ANGLE_OFFSET.times(-1)));
    }

    public void setModuleStateRaw(SwerveModuleState state) {
        state = optimizeState(state, new Rotation2d(getSteerPosition()));
        double drive_command = state.speedMetersPerSecond / SwerveConstants.MAX_MODULE_SPEED;
        driveMotor.set(drive_command * (motor_inv ? -1.0 : 1.0));

        double steercmd = steerPID.calculate(getSteerPosition(), state.angle.getRadians());
        steerMotor.setVoltage(12 * steercmd);
    }

    public void brakeMode() {
        SparkBaseConfig swerveMotorConfig = new SparkMaxConfig().idleMode(IdleMode.kBrake);

        driveMotor.configure(swerveMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        steerMotor.configure(swerveMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void coastMode() {
        SparkBaseConfig swerveMotorConfig = new SparkMaxConfig().idleMode(IdleMode.kBrake);

        driveMotor.configure(swerveMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        steerMotor.configure(swerveMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setModuleState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        setModuleStateRaw(state);
    }

    public void stop() {
        driveMotor.set(0);
        steerMotor.set(0);
    }

    private static SwerveModuleState optimizeState(SwerveModuleState desiredState, Rotation2d currentAngle) {
        double target = desiredState.angle.getRadians();
        double current = currentAngle.getRadians();

        double delta = Math.IEEEremainder(target - current, 2 * Math.PI);

        double speed = desiredState.speedMetersPerSecond;

        if (Math.abs(delta) > Math.PI / 2) {
            speed = -speed;
            delta = delta > 0 ? delta - Math.PI : delta + Math.PI;
        }

        double optimizedAngle = current + delta;
        return new SwerveModuleState(speed, new Rotation2d(optimizedAngle));
    }
}
