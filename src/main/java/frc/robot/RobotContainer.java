// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.SPI;

import frc.robot.NavX.AHRS;
import frc.robot.commands.*;
import frc.robot.constants.*;
import frc.robot.subsystems.*;

public class RobotContainer {
        private final AHRS gyro = new AHRS(SPI.Port.kMXP, (byte) 50);
        private final DriveSubsystem swerveSubsystem = new DriveSubsystem(gyro);
        private final ClimbingSubsystem climbSubsystem = new ClimbingSubsystem();
        private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
        private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
        private final DSVisionSubsystem dsVisionSubsystem = new DSVisionSubsystem();

        private final DriveCommand normalDrive;

        public RobotContainer() {
                normalDrive = new DriveCommand(swerveSubsystem,
                                new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT).getHID());
                                swerveSubsystem.setDefaultCommand(normalDrive);
                                new Controller(climbSubsystem, intakeSubsystem,
                                shooterSubsystem, swerveSubsystem, dsVisionSubsystem);
        }

        public void DisabledConfig() {
                swerveSubsystem.coasting();
        }

        public void EnableConfig() {
                swerveSubsystem.braked();
        }

        public Command getAutonomousCommand() {
                TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                                AutoConstants.kMaxSpeedMetersPerSecond,
                                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                .setKinematics(DriveConstants.KINEMATICS);

                Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(0, 0, new Rotation2d(0)),
                                List.of(new Translation2d(1, 0), new Translation2d(1, -1)),
                                new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
                                trajectoryConfig);

                PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
                PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
                ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0,
                                AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                                trajectory,
                                swerveSubsystem::getPose,
                                DriveConstants.KINEMATICS,
                                xController,
                                yController,
                                thetaController,
                                swerveSubsystem::setModuleStates,
                                swerveSubsystem);

                return new SequentialCommandGroup(
                                new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
                                swerveControllerCommand,
                                new InstantCommand(() -> swerveSubsystem.stopModules()));
        }
}
