// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootSpeaker extends Command {
  ShooterSubsystem shooter;

  public ShootSpeaker(ShooterSubsystem subsystem) {
    shooter = subsystem;
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    shooter.shootSpeaker();
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}