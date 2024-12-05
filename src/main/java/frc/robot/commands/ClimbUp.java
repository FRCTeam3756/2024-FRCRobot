// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbingSubsystem;

public class ClimbUp extends Command {
  ClimbingSubsystem subsystem;

  public ClimbUp(ClimbingSubsystem climb) {
    subsystem = climb;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    subsystem.ClimbingUp();
  }

  @Override
  public void end(boolean interrupted) {
    subsystem.ClimbingStop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}