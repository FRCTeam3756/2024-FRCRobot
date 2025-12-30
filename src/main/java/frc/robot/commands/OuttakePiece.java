// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class OuttakePiece extends Command {
  IntakeSubsystem outtake;

  public OuttakePiece(IntakeSubsystem subsystem) {
    outtake = subsystem;
    addRequirements(outtake);
  }

  @Override
  public void execute() {
    outtake.outtakeGamePiece();
  }

  @Override
  public void end(boolean interrupted) {
    outtake.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}