// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.constants.ControllerConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class Controller {    
    public final Trigger climbUpButton;
    public final Trigger climbDownButton;
    public final Trigger intakeButton;
    public final Trigger outtakeButton;
    public final Trigger shooterButton;
    // public final Trigger autoIntakeButton;
    // public final Trigger autoSpeakerButton;
    // public final Trigger autoClimbButton;
    // public final Trigger autoAmpButton;

    public Controller(ClimbingSubsystem climbSubsystem, IntakeSubsystem intakeSubsystem, 
                             ShooterSubsystem shooterSubsystem, DriveSubsystem swerveSubsystem, 
                             DSVisionSubsystem dsVisionSubsystem) {
                
        climbUpButton = ControllerConstants.climbUpButton;
        climbDownButton = ControllerConstants.climbDownButton;
        intakeButton = ControllerConstants.intakeButton;
        outtakeButton = ControllerConstants.outtakeButton;
        shooterButton = ControllerConstants.shooterButton;
        // autoIntakeButton = ControllerConstants.autoIntakeButton;
        // autoSpeakerButton = ControllerConstants.autoSpeakerButton;
        // autoClimbButton = ControllerConstants.autoClimbButton;
        // autoAmpButton = ControllerConstants.autoAmpButton;

        configureButtonBindings(climbSubsystem, intakeSubsystem, shooterSubsystem, swerveSubsystem, dsVisionSubsystem);
    }

    private void configureButtonBindings(ClimbingSubsystem climbSubsystem, IntakeSubsystem intakeSubsystem, 
                                         ShooterSubsystem shooterSubsystem, DriveSubsystem swerveSubsystem, 
                                         DSVisionSubsystem dsVisionSubsystem) {
        
        climbUpButton.whileTrue(new ClimbUp(climbSubsystem));
        climbDownButton.whileTrue(new ClimbDown(climbSubsystem));
        intakeButton.whileTrue(new IntakePiece(intakeSubsystem));
        outtakeButton.whileTrue(new OuttakePiece(intakeSubsystem));
        shooterButton.whileTrue(new ShootSpeaker(shooterSubsystem));
        // autoIntakeButton.whileTrue(new AINotePickup(swerveSubsystem, intakeSubsystem, dsVisionSubsystem));
        // autoSpeakerButton.whileTrue(new AISpeakerShot(...));
        // autoClimbButton.whileTrue(new AIClimbChain(...));
        // autoAmpButton.whileTrue(new AIAmpShot(...));
    }
}
