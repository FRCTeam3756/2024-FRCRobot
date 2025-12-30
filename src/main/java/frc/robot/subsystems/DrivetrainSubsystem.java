// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.studica.frc.AHRS;

import frc.robot.constants.*;

public class DrivetrainSubsystem extends SubsystemBase {

    SwerveModule frontLeft = new SwerveModule(SwerveConstants.FL_STEER_ID, SwerveConstants.FL_DRIVE_ID,
            SwerveConstants.FL_ABSOLUTE_ENCODER_PORT, SwerveConstants.FL_OFFSET_RADIANS,
            SwerveConstants.FL_ABSOLUTE_ENCODER_REVERSED,
            SwerveConstants.FL_MOTOR_REVERSED);

    SwerveModule frontRight = new SwerveModule(SwerveConstants.FR_STEER_ID, SwerveConstants.FR_DRIVE_ID,
            SwerveConstants.FR_ABSOLUTE_ENCODER_PORT, SwerveConstants.FR_OFFSET_RADIANS,
            SwerveConstants.FR_ABSOLUTE_ENCODER_REVERSED,
            SwerveConstants.FR_MOTOR_REVERSED);

    SwerveModule backRight = new SwerveModule(SwerveConstants.BR_STEER_ID, SwerveConstants.BR_DRIVE_ID,
            SwerveConstants.BR_ABSOLUTE_ENCODER_PORT, SwerveConstants.BR_OFFSET_RADIANS,
            SwerveConstants.BR_ABSOLUTE_ENCODER_REVERSED,
            SwerveConstants.BR_MOTOR_REVERSED);

    SwerveModule backLeft = new SwerveModule(SwerveConstants.BL_STEER_ID, SwerveConstants.BL_DRIVE_ID,
            SwerveConstants.BL_ABSOLUTE_ENCODER_PORT, SwerveConstants.BL_OFFSET_RADIANS,
            SwerveConstants.BL_ABSOLUTE_ENCODER_REVERSED,
            SwerveConstants.BL_MOTOR_REVERSED);

    private final AHRS gyro;
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(DriveConstants.KINEMATICS, new Rotation2d(),
            getModulePositions());

    public DrivetrainSubsystem(AHRS gyro) {
        this.gyro = gyro;
    }

    /**
     * Drives the robot at a specified speed and direction angle.
     *
     * @param speed Desired speed of the robot.
     * @param angle Desired direction angle in degrees.
     */
    public void drive(double speed, double angle) {
        angle += 90; // 90 degree offset from the gyro

        double radians = Math.toRadians(angle);

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                0.1 * (speed * Math.cos(radians)), // X velocity (forward/backward)
                speed * Math.sin(radians), // Y velocity (left/right)
                radians - getRotation2d().getRadians(),
                getRotation2d());

        SwerveModuleState[] moduleStates = DriveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(moduleStates);
    }

    public void coasting() {
        frontLeft.coastMode();
        frontRight.coastMode();
        backLeft.coastMode();
        backRight.coastMode();
    }

    public void braked() {
        frontLeft.brakeMode();
        frontRight.brakeMode();
        backLeft.brakeMode();
        backRight.brakeMode();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    @Override
    public void periodic() {
        odometry.update(getRotation2d(), getModulePositions());
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_MODULE_VELOCITY);
        frontLeft.setModuleState(desiredStates[DriveConstants.ModuleIndices.FRONT_LEFT]);
        frontRight.setModuleState(desiredStates[DriveConstants.ModuleIndices.FRONT_RIGHT]);
        backRight.setModuleState(desiredStates[DriveConstants.ModuleIndices.REAR_RIGHT]);
        backLeft.setModuleState(desiredStates[DriveConstants.ModuleIndices.REAR_LEFT]);
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = {
            frontLeft.getModulePosition(),
            frontRight.getModulePosition(),
            backLeft.getModulePosition(),
            backRight.getModulePosition()
        };
        return states;
    }
}
