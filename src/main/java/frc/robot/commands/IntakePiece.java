// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakePiece extends Command {
  IntakeSubsystem intake;

  public IntakePiece(IntakeSubsystem subystem) {
    intake = subystem;
    addRequirements(intake);
  }

  @Override
  public void execute() {
    intake.intakeGamePiece();
  }

  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}