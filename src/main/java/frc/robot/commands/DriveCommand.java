// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveCommand extends Command {

    private final DrivetrainSubsystem drivetrainSubsystem;
    private final XboxController xboxController;

    private final SlewRateLimiter speedRateLimiter = new SlewRateLimiter(2.0);

    private enum DriveState {
        Free,
        Locked
    };

    // Change if you don't want the robot to move
    private DriveState state = DriveState.Free;

    public DriveCommand(DrivetrainSubsystem drivetrainSubsystem, XboxController xboxController) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.xboxController = xboxController;

        speedRateLimiter.reset(SwerveConstants.SLOW_DRIVE_MULTIPLIER);

        addRequirements(drivetrainSubsystem);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(value, max));
    }

    public Translation2d applyDeadzone(Translation2d input) {
        double magnitude = input.getNorm();
        if (magnitude < ControllerConstants.DEADZONE) {
            return new Translation2d(0.0, 0.0);
        }

        Translation2d normalizedInput = input.div(magnitude);
        Translation2d scaledInput = normalizedInput.times((magnitude - ControllerConstants.DEADZONE) / (1.0 - ControllerConstants.DEADZONE));
        return new Translation2d(
                clamp(scaledInput.getX(), -1.0, 1.0),
                clamp(scaledInput.getY(), -1.0, 1.0));
    }

    public double applyDeadzone(double input) {
        return Math.abs(input) < ControllerConstants.DEADZONE ? 0.0 : (input - Math.signum(input) * ControllerConstants.DEADZONE) / (1.0 - ControllerConstants.DEADZONE);
    }

    @Override
    public void execute() {
        Translation2d joystickInput = new Translation2d(xboxController.getLeftX(), xboxController.getLeftY());
        Translation2d translationSpeed = applyDeadzone(joystickInput);
        double rotationSpeed = applyDeadzone(-xboxController.getRightX());
        double xSpeed = translationSpeed.getX();
        double ySpeed = translationSpeed.getY();

        xSpeed *= DriveConstants.XY_SPEED_LIMIT * DriveConstants.MAX_ROBOT_VELOCITY;
        ySpeed *= DriveConstants.XY_SPEED_LIMIT * DriveConstants.MAX_ROBOT_VELOCITY;
        rotationSpeed *= DriveConstants.Z_SPEED_LIMIT * DriveConstants.MAX_ROBOT_RADIAN_VELOCITY;

        double targetMultiplier = xboxController.getLeftStickButton() ? SwerveConstants.TURBO_DRIVE_MULTIPLIER : SwerveConstants.SLOW_DRIVE_MULTIPLIER;
        double speedMultiplier = speedRateLimiter.calculate(targetMultiplier);

        xSpeed *= speedMultiplier;
        ySpeed *= speedMultiplier;
        rotationSpeed *= speedMultiplier;

        if (xboxController.getXButton()) {
            drivetrainSubsystem.zeroHeading();
            drivetrainSubsystem.resetOdometry(new Pose2d());
        }

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                ySpeed, -xSpeed, -rotationSpeed,
                new Rotation2d(-drivetrainSubsystem.getRotation2d()
                        .rotateBy(DriveConstants.NAVX_ANGLE_OFFSET).getRadians()));

        switch (state) {
            case Free -> state = xboxController.getBButton() ? DriveState.Locked : DriveState.Free;
            case Locked -> state = ((joystickInput.getNorm() > 0.15) && !xboxController.getBButton()) ? DriveState.Free : DriveState.Locked;
        }

        SwerveModuleState[] calculatedModuleStates = DriveConstants.KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        drivetrainSubsystem.setModuleStates(calculatedModuleStates);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
