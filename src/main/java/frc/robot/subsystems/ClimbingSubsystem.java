// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbingSubsystem extends SubsystemBase {
  private Talon leftClaw = new Talon(2);
  private Talon rightClaw = new Talon(3);

  public void ClimbingUp() {
    leftClaw.set(-1);
    rightClaw.set(-1);
  }

  public void ClimbingDown() {
    leftClaw.set(1);
    rightClaw.set(1);
  }

  public void ClimbingStop() {
    leftClaw.set(0);
    rightClaw.set(0);
  }

  @Override
  public void periodic() {
  }
}