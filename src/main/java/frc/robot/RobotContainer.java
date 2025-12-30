// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.util.List;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.NavXUpdateRate;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommand;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class RobotContainer {

    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI, NavXUpdateRate.k50Hz);
    private final DrivetrainSubsystem drivetrainSubsystem = new DrivetrainSubsystem(gyro);

    private final DriveCommand normalDrive;

    public RobotContainer() {
        normalDrive = new DriveCommand(drivetrainSubsystem, new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT).getHID());
        drivetrainSubsystem.setDefaultCommand(normalDrive);
    }

    public void DisabledConfig() {
        drivetrainSubsystem.coasting();
    }

    public void EnableConfig() {
        drivetrainSubsystem.braked();
    }

    public Command getAutonomousCommand() {
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                SwerveConstants.MAX_MODULE_SPEED,
                AutoConstants.MAX_ACCELERATION)
                .setKinematics(DriveConstants.KINEMATICS);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(1, 0), new Translation2d(1, -1)),
                new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
                trajectoryConfig);

        PIDController xController = new PIDController(AutoConstants.P_X_CONTROLLER, 0, 0);
        PIDController yController = new PIDController(AutoConstants.P_Y_CONTROLLER, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.P_THETA_CONTROLLER, 0, 0,
                AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                drivetrainSubsystem::getPose,
                DriveConstants.KINEMATICS,
                xController,
                yController,
                thetaController,
                drivetrainSubsystem::setModuleStates,
                drivetrainSubsystem);

        return new SequentialCommandGroup(
                new InstantCommand(() -> drivetrainSubsystem.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> drivetrainSubsystem.stopModules()));
    }
}
