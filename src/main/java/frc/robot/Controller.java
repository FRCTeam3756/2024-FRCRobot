// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.IntakePiece;
import frc.robot.commands.OuttakePiece;
import frc.robot.commands.ShootSpeaker;
import frc.robot.constants.ControllerConstants;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class Controller {

    public final Trigger climbUpButton;
    public final Trigger climbDownButton;
    public final Trigger intakeButton;
    public final Trigger outtakeButton;
    public final Trigger shooterButton;

    public Controller(ClimbingSubsystem climbSubsystem, IntakeSubsystem intakeSubsystem,
            ShooterSubsystem shooterSubsystem, DrivetrainSubsystem drivetrainSubsystem) {

        climbUpButton = ControllerConstants.climbUpButton;
        climbDownButton = ControllerConstants.climbDownButton;
        intakeButton = ControllerConstants.intakeButton;
        outtakeButton = ControllerConstants.outtakeButton;
        shooterButton = ControllerConstants.shooterButton;

        configureButtonBindings(climbSubsystem, intakeSubsystem, shooterSubsystem);
    }

    private void configureButtonBindings(ClimbingSubsystem climbSubsystem, IntakeSubsystem intakeSubsystem,
            ShooterSubsystem shooterSubsystem) {

        climbUpButton.whileTrue(new ClimbUp(climbSubsystem));
        climbDownButton.whileTrue(new ClimbDown(climbSubsystem));
        intakeButton.whileTrue(new IntakePiece(intakeSubsystem));
        outtakeButton.whileTrue(new OuttakePiece(intakeSubsystem));
        shooterButton.whileTrue(new ShootSpeaker(shooterSubsystem));
    }
}
